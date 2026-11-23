// Feature benchmark v2 for Jetson TK1 (OpenCV4Tegra 2.4.10.1, CUDA 6.5).
// Question: does offloading work to the GPU free CPU time, and do CPU and GPU jobs run in parallel?
//  1) per-call wall time AND CPU time of the calling process for ORB, matching, LK optical flow, CPU vs GPU
//  2) concurrency: CPU ORB thread and GPU ORB thread at the same time, compared with 1 and 2 CPU threads
// Build: g++-4.8 -O2 feat_bench2.cpp -o feat_bench2 $(pkg-config --cflags --libs opencv) \
//          -I/usr/local/cuda-6.5/include -L/usr/local/cuda-6.5/lib -lcudart -lpthread -lrt
// Run:   LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib ./feat_bench2 recording_dir [frames] [blocking]
//   "blocking" = cudaDeviceScheduleBlockingSync (the CPU sleeps while the GPU works) instead of spinning.
#include <dirent.h>
#include <pthread.h>
#include <time.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <cuda_runtime.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/gpu/gpu.hpp>

static double wall_ms() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }
static double proc_cpu_ms() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }
static double thread_cpu_ms() { timespec t; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }

static std::vector<cv::Mat> g_gray;

// ---- 1) per-call timing -------------------------------------------------------------------
struct Acc {
  const char *name; double wall, cpu; int n;
  Acc(const char *s) : name(s), wall(0), cpu(0), n(0) {}
  void add(double w, double c) { wall += w; cpu += c; n++; }
  void print() const {
    if (!n) return;
    printf("%-44s wall %7.2f ms   CPU %7.2f ms (%3.0f %% of wall)\n", name, wall / n, cpu / n, 100.0 * cpu / wall);
  }
};

struct Timer {
  double w0, c0;
  void start() { w0 = wall_ms(); c0 = proc_cpu_ms(); }
  void stop(Acc &a) { a.add(wall_ms() - w0, proc_cpu_ms() - c0); }
};

static void per_call(int nframes) {
  const int WARM = 5;
  size_t N = std::min((size_t)nframes, g_gray.size());
  Acc orb_cpu("ORB 500 CPU"), orb_gpu("ORB 500 GPU (upload+download incl.)"), fast_gpu("FAST GPU (upload+download incl.)");
  Acc match_cpu("match 500x500 Hamming CPU"), match_gpu("match 500x500 Hamming GPU (+download)");
  Acc lk_cpu("LK optical flow 400 pts CPU"), lk_gpu("LK optical flow 400 pts GPU (+up/download)");

  cv::ORB orb(500, 1.2f, 8);
  cv::gpu::ORB_GPU orb_g(500);
  cv::gpu::FAST_GPU fast_g(20);
  cv::BFMatcher bf(cv::NORM_HAMMING, false);
  cv::gpu::BruteForceMatcher_GPU_base bf_g(cv::gpu::BruteForceMatcher_GPU_base::HammingDist);
  cv::gpu::PyrLKOpticalFlow lk_g;
  lk_g.winSize = cv::Size(21, 21);
  lk_g.maxLevel = 3;
  lk_g.iters = 30;

  Timer t;
  for (size_t i = 0; i + 1 < N; i++) {
    const cv::Mat &a = g_gray[i], &b = g_gray[i + 1];
    bool rec = (int)i >= WARM;

    std::vector<cv::KeyPoint> ka, kb;
    cv::Mat da, db;
    t.start(); orb(a, cv::Mat(), ka, da); if (rec) t.stop(orb_cpu);
    orb(b, cv::Mat(), kb, db);

    cv::gpu::GpuMat up, kg, dg, up2, kg2, dg2;
    std::vector<cv::KeyPoint> kgv;
    t.start();
    up.upload(a);
    orb_g(up, cv::gpu::GpuMat(), kg, dg);
    orb_g.downloadKeyPoints(kg, kgv);
    if (rec) t.stop(orb_gpu);

    t.start();
    cv::gpu::GpuMat fk;
    up.upload(a);
    fast_g(up, cv::gpu::GpuMat(), fk);
    std::vector<cv::KeyPoint> fkv;
    fast_g.downloadKeypoints(fk, fkv);
    if (rec) t.stop(fast_gpu);

    // matching (descriptors a -> b)
    std::vector<cv::DMatch> m;
    t.start(); bf.match(da, db, m); if (rec) t.stop(match_cpu);
    up2.upload(b);
    orb_g(up2, cv::gpu::GpuMat(), kg2, dg2);
    {
      std::vector<cv::DMatch> mg;
      t.start();
      bf_g.match(dg, dg2, mg);
      if (rec) t.stop(match_gpu);
    }

    // LK optical flow on FAST corners
    std::vector<cv::KeyPoint> corners;
    cv::FAST(a, corners, 25, true);
    std::vector<cv::Point2f> p0, p1;
    for (size_t k = 0; k < corners.size() && p0.size() < 400; k++) p0.push_back(corners[k].pt);
    if (p0.size() < 20) continue;
    std::vector<uchar> st;
    std::vector<float> er;
    t.start();
    cv::calcOpticalFlowPyrLK(a, b, p0, p1, st, er, cv::Size(21, 21), 3);
    if (rec) t.stop(lk_cpu);

    cv::Mat p0m(1, (int)p0.size(), CV_32FC2, &p0[0]);
    cv::gpu::GpuMat pg, ng, sg, eg, ag, bg;
    t.start();
    ag.upload(a); bg.upload(b); pg.upload(p0m);
    lk_g.sparse(ag, bg, pg, ng, sg, &eg);
    cv::Mat nm; ng.download(nm); cv::Mat sm; sg.download(sm);
    if (rec) t.stop(lk_gpu);
  }
  printf("\n== per-call cost (CPU = CPU time of this process while the call runs) ==\n");
  orb_cpu.print(); orb_gpu.print(); fast_gpu.print();
  match_cpu.print(); match_gpu.print();
  lk_cpu.print(); lk_gpu.print();
}

// ---- 2) concurrency ------------------------------------------------------------------------
struct Job {
  int kind;          // 0 = CPU ORB, 1 = GPU ORB
  double seconds;
  int frames;
  double cpu_ms;     // CPU time of this thread
  double wall_ms;
};

static void *worker(void *arg) {
  Job *j = (Job *)arg;
  size_t N = g_gray.size();
  double w0 = wall_ms(), c0 = thread_cpu_ms();
  int n = 0;
  if (j->kind == 0) {
    cv::ORB orb(500, 1.2f, 8);
    std::vector<cv::KeyPoint> k; cv::Mat d;
    while (wall_ms() - w0 < j->seconds * 1000) { orb(g_gray[n % N], cv::Mat(), k, d); n++; }
  } else {
    cv::gpu::setDevice(0);
    cv::gpu::ORB_GPU orb_g(500);
    cv::gpu::GpuMat up, kg, dg;
    std::vector<cv::KeyPoint> kv;
    while (wall_ms() - w0 < j->seconds * 1000) {
      up.upload(g_gray[n % N]);
      orb_g(up, cv::gpu::GpuMat(), kg, dg);
      orb_g.downloadKeyPoints(kg, kv);
      n++;
    }
  }
  j->frames = n; j->cpu_ms = thread_cpu_ms() - c0; j->wall_ms = wall_ms() - w0;
  return 0;
}

static void run(const char *label, std::vector<int> kinds, double seconds) {
  std::vector<Job> jobs(kinds.size());
  std::vector<pthread_t> th(kinds.size());
  for (size_t i = 0; i < kinds.size(); i++) { jobs[i].kind = kinds[i]; jobs[i].seconds = seconds; jobs[i].frames = 0; }
  double p0 = proc_cpu_ms(), w0 = wall_ms();
  for (size_t i = 0; i < kinds.size(); i++) pthread_create(&th[i], 0, worker, &jobs[i]);
  for (size_t i = 0; i < kinds.size(); i++) pthread_join(th[i], 0);
  double pw = wall_ms() - w0, pc = proc_cpu_ms() - p0;
  printf("%-32s", label);
  for (size_t i = 0; i < kinds.size(); i++)
    printf("  [%s: %5.1f fps, thread CPU %3.0f %%]", jobs[i].kind ? "GPU-ORB" : "CPU-ORB", jobs[i].frames * 1000.0 / jobs[i].wall_ms,
           100.0 * jobs[i].cpu_ms / jobs[i].wall_ms);
  printf("  process CPU %.0f %% of one core\n", 100.0 * pc / pw);
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s dir [frames] [blocking]\n", argv[0]); return 1; }
  int nframes = argc > 2 ? atoi(argv[2]) : 60;
  bool blocking = argc > 3 && !strcmp(argv[3], "blocking");
  if (blocking) cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
  printf("CUDA wait mode: %s\n", blocking ? "blocking sync (CPU sleeps)" : "default (may spin)");

  std::string dir = argv[1];
  std::vector<std::string> files;
  DIR *d = opendir(dir.c_str());
  if (!d) { perror("opendir"); return 1; }
  while (dirent *e = readdir(d)) {
    std::string n = e->d_name;
    if (n.size() > 4 && n[0] == 'r' && n[1] == '-' && n.substr(n.size() - 4) == ".ppm") files.push_back(dir + "/" + n);
  }
  closedir(d);
  std::sort(files.begin(), files.end());
  if ((int)files.size() > nframes) files.resize(nframes);
  for (size_t i = 0; i < files.size(); i++) {
    cv::Mat bgr = cv::imread(files[i], cv::IMREAD_COLOR), g;
    if (bgr.empty()) continue;
    cv::cvtColor(bgr, g, CV_BGR2GRAY);
    g_gray.push_back(g);
  }
  printf("loaded %zu frames\n", g_gray.size());
  if (g_gray.size() < 20) return 1;
  cv::gpu::setDevice(0);

  per_call(nframes);

  printf("\n== concurrency, 10 s each (4 CPU cores available) ==\n");
  run("1 CPU thread", std::vector<int>(1, 0), 10);
  run("GPU thread alone", std::vector<int>(1, 1), 10);
  std::vector<int> k;
  k.push_back(0); k.push_back(1);
  run("CPU thread + GPU thread", k, 10);
  std::vector<int> k2;
  k2.push_back(0); k2.push_back(0);
  run("2 CPU threads", k2, 10);
  return 0;
}
