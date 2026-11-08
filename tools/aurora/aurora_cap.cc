// Headless Aurora930 capture using the vendor stream SDK (aarch64).
// Writes the latest RGB + raw 16-bit depth frame atomically to --out, plus meta.json (timestamps, intrinsics).
//   aurora_cap --probe
//   aurora_cap --mode N --fps F --out DIR [--min 150 --max 4000] [--align 1]
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <sys/stat.h>
#include "deptrum/device.h"
#include "deptrum/stream.h"
#include "deptrum/aurora900_series.h"
using namespace deptrum;
using namespace deptrum::stream;

static std::atomic<bool> g_run{true};
static void on_sig(int) { g_run = false; }

static double wall() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
static void write_atomic(const std::string& path, const void* d, size_t n) {
  std::string tmp = path + ".tmp";
  FILE* f = fopen(tmp.c_str(), "wb");
  if (!f) return;
  fwrite(d, 1, n, f);
  fclose(f);
  rename(tmp.c_str(), path.c_str());
}

int main(int argc, char** argv) {
  int mode_idx = -1, fps = 5, align = 1, vmin = 150, vmax = 4000;
  bool probe = false;
  std::string out = "/tmp/aurora";
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--probe") probe = true;
    else if (a == "--mode" && i + 1 < argc) mode_idx = atoi(argv[++i]);
    else if (a == "--fps" && i + 1 < argc) fps = atoi(argv[++i]);
    else if (a == "--out" && i + 1 < argc) out = argv[++i];
    else if (a == "--align" && i + 1 < argc) align = atoi(argv[++i]);
    else if (a == "--min" && i + 1 < argc) vmin = atoi(argv[++i]);
    else if (a == "--max" && i + 1 < argc) vmax = atoi(argv[++i]);
  }
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);

  std::vector<DeviceInformation> list;
  DeviceManager::GetInstance()->GetDeviceList(list);
  if (list.empty()) { fprintf(stderr, "no device\n"); return 2; }
  auto dev = DeviceManager::GetInstance()->CreateDevice(list[0]);
  if (!dev || dev->Open() != 0) { fprintf(stderr, "open failed\n"); return 3; }
  auto a9 = std::dynamic_pointer_cast<Aurora900>(dev);

  std::vector<std::tuple<FrameMode, FrameMode, FrameMode>> modes;
  dev->GetSupportedFrameMode(modes);
  if (probe) {
    for (size_t i = 0; i < modes.size(); i++)
      printf("mode %zu: ir=%d rgb=%d depth=%d\n", i, (int)std::get<0>(modes[i]), (int)std::get<1>(modes[i]), (int)std::get<2>(modes[i]));
    std::vector<StreamType> st;
    dev->GetSupportedStreamType(st);
    for (auto s : st) printf("stream type %d\n", (int)s);
    Intrinsic ii, ri; Extrinsic ex;
    dev->GetCameraParameters(ii, ri, ex);
    printf("ir  %dx%d f=%.2f,%.2f c=%.2f,%.2f\n", ii.cols, ii.rows, ii.focal_length[0], ii.focal_length[1], ii.principal_point[0], ii.principal_point[1]);
    printf("rgb %dx%d f=%.2f,%.2f c=%.2f,%.2f\n", ri.cols, ri.rows, ri.focal_length[0], ri.focal_length[1], ri.principal_point[0], ri.principal_point[1]);
    dev->Close();
    return 0;
  }
  if (mode_idx < 0 || mode_idx >= (int)modes.size()) { fprintf(stderr, "bad --mode\n"); return 4; }
  dev->SetMode(std::get<0>(modes[mode_idx]), std::get<1>(modes[mode_idx]), std::get<2>(modes[mode_idx]));
  if (a9) {
    a9->SetIrFps(fps);
    a9->SwitchAlignedMode(align == 1);
    if (align == 1) a9->DepthCorrection(true);
    a9->FilterOutRangeDepthMap(vmin, vmax);
  }
  Intrinsic ii, ri; Extrinsic ex;
  dev->GetCameraParameters(ii, ri, ex);

  mkdir(out.c_str(), 0775);
  Stream* stream = nullptr;
  std::vector<StreamType> types = {StreamType::kRgbd};  // aligned RGB + depth
  if (dev->CreateStream(stream, types) != 0 || stream->Start() != 0) { fprintf(stderr, "stream failed\n"); return 5; }

  long n = 0, errs = 0;
  double t0 = wall();
  while (g_run) {
    StreamFrames fr;
    int r = stream->GetFrames(fr, 2000);
    if (r != 0) { errs++; fprintf(stderr, "GetFrames err %d (%ld)\n", r, errs); if (errs > 20) break; continue; }
    double tw = wall();
    char meta[1024];
    int rgb_n = 0, dep_n = 0, rc = 0, rr = 0, dc = 0, dr = 0, rfmt = 0, dbits = 0;
    unsigned long long rts = 0, dts = 0;
    std::vector<char> rgbv, depv;
    for (int i = 0; i < fr.count; i++) {
      auto f = fr.frame_ptr[i];
      if (!f || !f->data) continue;
      if (f->frame_type == FrameType::kDepthFrame) {
        depv.assign((char*)f->data.get(), (char*)f->data.get() + f->size);
        dep_n = f->size; dc = f->cols; dr = f->rows; dts = f->timestamp; dbits = f->bits_per_pixel;
      } else if (f->frame_type == FrameType::kRgbFrame) {
        rgbv.assign((char*)f->data.get(), (char*)f->data.get() + f->size);
        rgb_n = f->size; rc = f->cols; rr = f->rows; rts = f->timestamp; rfmt = (int)f->image_format;
      }
    }
    n++;
    snprintf(meta, sizeof meta,
             "{\"seq\":%ld,\"wall\":%.4f,\"rgb\":{\"bytes\":%d,\"cols\":%d,\"rows\":%d,\"format\":%d,\"ts\":%llu},"
             "\"depth\":{\"bytes\":%d,\"cols\":%d,\"rows\":%d,\"bits\":%d,\"ts\":%llu,\"units\":\"mm\"},"
             "\"intrinsics\":{\"ir\":{\"cols\":%d,\"rows\":%d,\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f},"
             "\"rgb\":{\"cols\":%d,\"rows\":%d,\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f}},\"aligned\":%d,\"errors\":%ld}",
             n, tw, rgb_n, rc, rr, rfmt, rts, dep_n, dc, dr, dbits, dts,
             ii.cols, ii.rows, ii.focal_length[0], ii.focal_length[1], ii.principal_point[0], ii.principal_point[1],
             ri.cols, ri.rows, ri.focal_length[0], ri.focal_length[1], ri.principal_point[0], ri.principal_point[1], align, errs);
    uint32_t ml = strlen(meta);
    std::vector<char> blob(4 + ml + rgbv.size() + depv.size());
    memcpy(blob.data(), &ml, 4);
    memcpy(blob.data() + 4, meta, ml);
    if (!rgbv.empty()) memcpy(blob.data() + 4 + ml, rgbv.data(), rgbv.size());
    if (!depv.empty()) memcpy(blob.data() + 4 + ml + rgbv.size(), depv.data(), depv.size());
    write_atomic(out + "/frame.bin", blob.data(), blob.size());
    if (n % 25 == 0) fprintf(stderr, "frames=%ld fps=%.2f errs=%ld\n", n, n / (tw - t0), errs);
  }
  stream->Stop();
  dev->DestroyStream(stream);
  dev->Close();
  return 0;
}
