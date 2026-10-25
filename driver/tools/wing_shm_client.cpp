// wing_shm_client - minimal stand-in for the wing-sim daemon side of wing_shm.
//
// Attaches to the driver's shared memory, and for every frame the DAW plays on
// "USB In" c, writes gain * sample to "USB Out" c (the DAW's record inputs).
// Default gain is -1 (polarity invert) so the effect is easy to verify:
// with the client running, a loopback recording comes back inverted; stop the
// client and the driver falls back to plain loopback within 100 ms.
//
//   wing_shm_client [--name /wingsim.audio.v1] [--gain -1.0] [--seconds 0]
//                   [--create] [--status]
//
// NOT production code (T020/T022 replace it): polls every 250 us instead of
// being woken by the driver, runs on a normal-priority thread.

#include "wing_shm.h"

#include <mach/mach_time.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int)
{
    g_stop = true;
}

void Usage()
{
    std::fprintf(stderr,
        "usage: wing_shm_client [--name NAME] [--gain G] [--seconds S] [--create] "
        "[--status]\n");
}

wing_shm_header* Attach(const char* name, bool create)
{
    int fd = shm_open(name, O_RDWR, 0);
    bool fresh = false;
    if (fd < 0 && errno == ENOENT && create) {
        // Normally the driver creates the segment. --create lets the daemon
        // start first (or run with no driver at all for tests).
        fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0666);
        if (fd >= 0) {
            if (ftruncate(fd, (off_t)WING_SHM_TOTAL_BYTES) != 0) {
                std::perror("ftruncate");
                close(fd);
                shm_unlink(name);
                return nullptr;
            }
            fresh = true;
        }
    }
    if (fd < 0) {
        std::fprintf(stderr, "shm_open(%s): %s%s\n", name, std::strerror(errno),
            errno == ENOENT ? " (driver not loaded? use --create to create it)" : "");
        return nullptr;
    }
    struct stat st{};
    fstat(fd, &st);
    void* p = mmap(nullptr, WING_SHM_TOTAL_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) {
        std::perror("mmap");
        return nullptr;
    }
    auto* h = static_cast<wing_shm_header*>(p);
    if (fresh) {
        wing_shm_init_header(h, 48000);
    }
    if (int rc = wing_shm_validate_header(h, (uint64_t)st.st_size); rc != 0) {
        std::fprintf(stderr, "incompatible segment (rc=%d, version=%u, size=%lld)\n", rc,
            h->version, (long long)st.st_size);
        munmap(p, WING_SHM_TOTAL_BYTES);
        return nullptr;
    }
    return h;
}

double TicksToMs(uint64_t ticks)
{
    mach_timebase_info_data_t tb{};
    mach_timebase_info(&tb);
    return (double)ticks * tb.numer / tb.denom / 1e6;
}

void PrintStatus(const wing_shm_header* h)
{
    const uint64_t now = mach_absolute_time();
    auto age = [&](uint64_t hb) -> std::string {
        if (hb == 0) return "never";
        if (hb > now) return "0 ms";
        char b[32];
        std::snprintf(b, sizeof b, "%.1f ms ago", TicksToMs(now - hb));
        return b;
    };
    std::printf("segment        v%u, %u ch, ring %u frames, input delay %u frames\n",
        h->version, h->channels, h->ring_frames, h->input_delay_frames);
    std::printf("driver         pid %llu, rate %u, epoch %u, running %u, loopback %u, "
                "heartbeat %s\n",
        (unsigned long long)WING_LOAD_RLX(&h->driver_pid), WING_LOAD_RLX(&h->sample_rate),
        WING_LOAD_RLX(&h->io_epoch), WING_LOAD_RLX(&h->io_running),
        WING_LOAD_RLX(&h->loopback_active), age(WING_LOAD_RLX(&h->driver_heartbeat)).c_str());
    std::printf("daemon         pid %llu, heartbeat %s\n",
        (unsigned long long)WING_LOAD_RLX(&h->daemon_pid),
        age(WING_LOAD_RLX(&h->daemon_heartbeat)).c_str());
    std::printf("to_daemon      write_time %lld, read_time %lld, epoch %u, overruns %u\n",
        (long long)WING_LOAD_RLX(&h->to_daemon.write_time),
        (long long)WING_LOAD_RLX(&h->to_daemon.read_time), WING_LOAD_RLX(&h->to_daemon.epoch),
        WING_LOAD_RLX(&h->to_daemon.overruns));
    std::printf("from_daemon    write_time %lld, read_time %lld, epoch %u, underruns %u\n",
        (long long)WING_LOAD_RLX(&h->from_daemon.write_time),
        (long long)WING_LOAD_RLX(&h->from_daemon.read_time),
        WING_LOAD_RLX(&h->from_daemon.epoch), WING_LOAD_RLX(&h->from_daemon.underruns));
}

} // namespace

int main(int argc, char** argv)
{
    const char* name = WING_SHM_NAME;
    float gain = -1.0f;
    double seconds = 0;
    bool create = false;
    bool status = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--name" && i + 1 < argc) {
            name = argv[++i];
        } else if (a == "--gain" && i + 1 < argc) {
            gain = std::strtof(argv[++i], nullptr);
        } else if (a == "--seconds" && i + 1 < argc) {
            seconds = std::strtod(argv[++i], nullptr);
        } else if (a == "--create") {
            create = true;
        } else if (a == "--status") {
            status = true;
        } else {
            Usage();
            return 2;
        }
    }

    wing_shm_header* h = Attach(name, create);
    if (!h) {
        return 1;
    }
    if (status) {
        PrintStatus(h);
        return 0;
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    const uint32_t n = h->ring_frames;
    const uint32_t chans = h->channels;
    const int64_t maxLag = (int64_t)n / 2;
    const float* in = wing_shm_ring(h, h->to_daemon_offset);
    float* out = wing_shm_ring(h, h->from_daemon_offset);

    WING_STORE_REL(&h->daemon_pid, (uint64_t)getpid());
    std::fprintf(stderr, "wing_shm_client: attached %s, gain %.3f, %u ch @ %u Hz\n", name,
        gain, chans, WING_LOAD_RLX(&h->sample_rate));

    uint32_t epoch = ~0u;
    int64_t cursor = 0;
    const uint64_t start = mach_absolute_time();
    uint64_t lastReport = start;

    while (!g_stop) {
        const uint64_t now = mach_absolute_time();
        WING_STORE_REL(&h->daemon_heartbeat, now);

        const uint32_t e = WING_LOAD_ACQ(&h->io_epoch);
        if (e != epoch) {
            // New driver IO session: sample time restarted. Skip any backlog.
            epoch = e;
            cursor = WING_LOAD_ACQ(&h->to_daemon.write_time);
            WING_STORE_REL(&h->from_daemon.write_time, cursor);
            WING_STORE_REL(&h->from_daemon.epoch, e);
        }

        const int64_t w = WING_LOAD_ACQ(&h->to_daemon.write_time);
        if (w < cursor) {
            cursor = w; // should not happen within an epoch
        }
        if (w - cursor > maxLag) {
            WING_INC_RLX(&h->to_daemon.overruns);
            cursor = w - 512;
        }
        for (int64_t t = cursor; t < w; t++) {
            const uint32_t idx = wing_shm_ring_index(t, n);
            for (uint32_t c = 0; c < chans; c++) {
                out[(size_t)c * n + idx] = gain * in[(size_t)c * n + idx];
            }
        }
        if (w != cursor) {
            WING_STORE_REL(&h->from_daemon.write_time, w);
            WING_STORE_RLX(&h->to_daemon.read_time, w);
            cursor = w;
        }

        if (seconds > 0 && TicksToMs(now - start) > seconds * 1000.0) {
            break;
        }
        if (TicksToMs(now - lastReport) > 5000) {
            lastReport = now;
            std::fprintf(stderr, "wing_shm_client: t=%lld epoch=%u underruns=%u overruns=%u\n",
                (long long)w, e, WING_LOAD_RLX(&h->from_daemon.underruns),
                WING_LOAD_RLX(&h->to_daemon.overruns));
        }
        usleep(250);
    }

    // Detach: driver switches back to loopback immediately.
    WING_STORE_REL(&h->daemon_heartbeat, (uint64_t)0);
    WING_STORE_REL(&h->daemon_pid, (uint64_t)0);
    std::fprintf(stderr, "wing_shm_client: detached\n");
    return 0;
}
