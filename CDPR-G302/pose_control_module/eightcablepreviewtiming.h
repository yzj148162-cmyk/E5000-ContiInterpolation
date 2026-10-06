#ifndef EIGHTCABLEPREVIEWTIMING_H
#define EIGHTCABLEPREVIEWTIMING_H

#include <array>
#include <chrono>
#include <cstdint>

// Diagnostic only. Primary sections partition totalNs; nested sections are
// INCLUDED in Controller/Allocation and must not be added to the primary sum.
struct EightCablePreviewTiming {
    enum Part {
        Setup, ForwardKinematics, Residual, Jacobian, Velocity,
        ReferenceDynamics, Controller, AllocationDiagnostics, Allocation, Finalize,
        ControllerValidation, ControllerMassCheck, ControllerModalDamping,
        AllocatorSvd, AllocatorTensionPolygon, AllocatorBoundedPolygon, Count
    };
    static constexpr int primaryCount = ControllerValidation;
    std::array<std::int64_t, Count> elapsedNs{};
    std::int64_t totalNs = 0;
    std::uint64_t measuredMask = 0;
    int lastPart = Setup;

    double microseconds(int part) const { return elapsedNs[part] / 1000.0; }
    static const char* column(int part) {
        static const char* const names[Count] = {
            "preview_setup_us", "preview_fk_us", "preview_residual_us",
            "preview_jacobian_us", "preview_velocity_us", "preview_reference_dynamics_us",
            "preview_controller_us", "preview_allocation_diagnostics_us",
            "preview_allocator_us", "preview_finalize_us",
            "preview_controller_validation_us", "preview_controller_mass_check_us",
            "preview_controller_modal_damping_us", "preview_allocator_svd_us",
            "preview_allocator_tension_polygon_us", "preview_allocator_bounded_polygon_us"
        };
        return names[part];
    }
};

class EightCablePreviewClock {
    using Clock = std::chrono::steady_clock;
public:
    explicit EightCablePreviewClock(EightCablePreviewTiming& value)
        : value_(value), started_(Clock::now()), previous_(started_) {
        value_ = {};
        value_.measuredMask = 1;
    }
    void next(EightCablePreviewTiming::Part part) {
        const auto now = Clock::now();
        value_.elapsedNs[current_] +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - previous_).count();
        previous_ = now;
        current_ = part;
        value_.lastPart = part;
        value_.measuredMask |= std::uint64_t(1) << part;
    }
    // Called BEFORE copying the StartPreview on both success and failure.
    void finish() {
        const auto now = Clock::now();
        value_.elapsedNs[current_] +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - previous_).count();
        value_.totalNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now - started_).count();
    }
private:
    EightCablePreviewTiming& value_;
    Clock::time_point started_, previous_;
    int current_ = EightCablePreviewTiming::Setup;
};

// Optional inner timings. Normal runtime calls pass nullptr and do not read
// the clock. Early returns still finish the section in the caller's timing object.
class EightCablePreviewSection {
    using Clock = std::chrono::steady_clock;
public:
    EightCablePreviewSection(EightCablePreviewTiming* value, EightCablePreviewTiming::Part part)
        : value_(value), part_(part) {
        if(value_) {
            started_ = Clock::now();
            value_->measuredMask |= std::uint64_t(1) << part_;
        }
    }
    ~EightCablePreviewSection() { finish(); }
    void finish() {
        if(value_) {
            value_->elapsedNs[part_] +=
                    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - started_).count();
            value_ = nullptr;
        }
    }
    EightCablePreviewSection(const EightCablePreviewSection&) = delete;
    EightCablePreviewSection& operator=(const EightCablePreviewSection&) = delete;
private:
    EightCablePreviewTiming* value_;
    int part_;
    Clock::time_point started_{};
};

#endif

