// Feature detection benchmark for Jetson TK1 (OpenCV4Tegra 2.4.10.1, CUDA 6.5).
// Reads Kinect RGB frames (r-*.ppm from fakenect-record) and times FAST and ORB on CPU and GPU.
// Build: g++-4.8 -O2 feat_bench.cpp -o feat_bench $(pkg-config --cflags --libs opencv)
// Run:   ./feat_bench /path/to/recording [max_frames]
#include <dirent.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

static double now_ms() { return 1000.0 * cv::getTickCount() / cv::getTickFrequency(); }

struct Stat {
  const char *name;
  std::vector<double> ms;
  double kp_sum;
  Stat(const char *n) : name(n), kp_sum(0) {}
  void add(double t, size_t kp) { ms.push_back(t); kp_sum += kp; }
  void print() const {
    if (ms.empty()) { printf("%-34s no data\n", name); return; }
    std::vector<double> s(ms);
    std::sort(s.begin(), s.end());
    double sum = 0;
    for (size_t i = 0; i < s.size(); i++) sum += s[i];
    double mean = sum / s.size();
    printf("%-34s mean %7.2f ms  median %7.2f  p95 %7.2f  => %6.1f fps   avg keypoints %6.1f\n", name, mean,
           s[s.size() / 2], s[(size_t)(s.size() * 0.95)], 1000.0 / mean, kp_sum / s.size());
  }
};

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s recording_dir [max_frames]\n", argv[0]); return 1; }
  std::string dir = argv[1];
  size_t max_frames = argc > 2 ? atoi(argv[2]) : 200;

  std::vector<std::string> files;
  DIR *d = opendir(dir.c_str());
  if (!d) { perror("opendir"); return 1; }
  while (dirent *e = readdir(d)) {
    std::string n = e->d_name;
    if (n.size() > 4 && n[0] == 'r' && n[1] == '-' && n.substr(n.size() - 4) == ".ppm") files.push_back(dir + "/" + n);
  }
  closedir(d);
  std::sort(files.begin(), files.end());
  if (files.size() > max_frames) files.resize(max_frames);
  printf("frames: %zu from %s\n", files.size(), dir.c_str());

  // Preload and convert to gray so that disk and color conversion are not measured.
  std::vector<cv::Mat> gray, gray_small;
  for (size_t i = 0; i < files.size(); i++) {
    cv::Mat bgr = cv::imread(files[i], cv::IMREAD_COLOR);
    if (bgr.empty()) continue;
    cv::Mat g, gs;
    cv::cvtColor(bgr, g, CV_BGR2GRAY);
    cv::resize(g, gs, cv::Size(320, 240), 0, 0, cv::INTER_AREA);
    gray.push_back(g);
    gray_small.push_back(gs);
  }
  if (gray.empty()) { fprintf(stderr, "no frames loaded\n"); return 1; }
  printf("loaded %zu frames of %dx%d\n", gray.size(), gray[0].cols, gray[0].rows);

  int gpus = cv::gpu::getCudaEnabledDeviceCount();
  printf("CUDA devices seen by OpenCV: %d\n", gpus);

  const int WARM = 5;
  Stat fast_cpu("FAST (CPU) 640x480"), orb_cpu("ORB 500 (CPU) 640x480"), orb_cpu_small("ORB 500 (CPU) 320x240");
  Stat fast_gpu_s("FAST (GPU, incl. up/download)"), orb_gpu_s("ORB 500 (GPU, incl. up/download)");

  cv::ORB orb(500, 1.2f, 8);
  cv::ORB orb_s(500, 1.2f, 8);

  for (size_t i = 0; i < gray.size(); i++) {
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    double t0 = now_ms();
    cv::FAST(gray[i], kp, 20, true);
    double t1 = now_ms();
    if ((int)i >= WARM) fast_cpu.add(t1 - t0, kp.size());

    kp.clear();
    t0 = now_ms();
    orb(gray[i], cv::Mat(), kp, desc);
    t1 = now_ms();
    if ((int)i >= WARM) orb_cpu.add(t1 - t0, kp.size());

    kp.clear();
    t0 = now_ms();
    orb_s(gray_small[i], cv::Mat(), kp, desc);
    t1 = now_ms();
    if ((int)i >= WARM) orb_cpu_small.add(t1 - t0, kp.size());
  }

  if (gpus > 0) {
    cv::gpu::FAST_GPU fast_g(20);
    cv::gpu::ORB_GPU orb_g(500);
    for (size_t i = 0; i < gray.size(); i++) {
      cv::gpu::GpuMat up, kp_g, desc_g;
      std::vector<cv::KeyPoint> kp;
      double t0 = now_ms();
      up.upload(gray[i]);
      fast_g(up, cv::gpu::GpuMat(), kp_g);
      fast_g.downloadKeypoints(kp_g, kp);
      double t1 = now_ms();
      if ((int)i >= WARM) fast_gpu_s.add(t1 - t0, kp.size());

      kp.clear();
      cv::gpu::GpuMat kp_g2;
      t0 = now_ms();
      up.upload(gray[i]);
      orb_g(up, cv::gpu::GpuMat(), kp_g2, desc_g);
      orb_g.downloadKeyPoints(kp_g2, kp);
      cv::Mat desc;
      desc_g.download(desc);
      t1 = now_ms();
      if ((int)i >= WARM) orb_gpu_s.add(t1 - t0, kp.size());
    }
  }

  printf("\n(first %d frames ignored as warm-up)\n", WARM);
  fast_cpu.print();
  orb_cpu.print();
  orb_cpu_small.print();
  fast_gpu_s.print();
  orb_gpu_s.print();
  return 0;
}
