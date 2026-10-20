#pragma once
// exp/perf-patch: upstream's feature/performance-patch-1 includes this header and lists
// src/lib/frame_timing.cpp, but never committed either. Only this call is used.
void flushGuestPresentationTimings();
