# Runtime Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Harden asynchronous matching, camera acquisition, registration validation, tests, and documentation while preserving current matching behavior.

**Architecture:** A small request tracker separates input lifetime from worker lifetime. Controllers transport request ids, the main window rejects stale callbacks, the camera worker owns its SDK state and synchronization, and the matcher fails early on invalid intermediate data.

**Tech Stack:** C++17, Qt 6, PCL 1.15, Eigen, OpenCascade, Visual Studio/MSBuild, Hikvision MVS SDK.

## Global Constraints

- Keep the current registration thresholds and CAD-to-scan transform convention.
- Keep rigid seam mapping in `WeldSeamService`.
- Do not require physical camera hardware in automated tests.
- Do not commit or push without a separate user request.

---

### Task 1: Request lifetime tests and tracker

**Files:**
- Create: `dianyun/MatchRequestTracker.h`
- Modify: `tests/WeldSeamMappingTests.cpp`

**Interfaces:**
- Produces: `MatchRequestTracker::invalidateData()`, `beginRequest()`, `accepts(id)`, and `complete(id)`.

- [x] Add tests proving a current request is accepted, completion rejects it, and data replacement rejects an old request.
- [x] Run the test build and confirm the new test fails before the tracker exists.
- [x] Implement the header-only tracker with monotonically increasing `std::uint64_t` ids and revisions.
- [x] Re-run the tests and confirm the tracker cases pass.

### Task 2: Match callback versioning and UI state

**Files:**
- Modify: `dianyun/ModelMatchController.h`
- Modify: `dianyun/ModelMatchController.cpp`
- Modify: `dianyun/dianyun.h`
- Modify: `dianyun/dianyun.cpp`

**Interfaces:**
- Consumes: `MatchRequestTracker`.
- Produces: controller signals `progress(requestId, stage)`, `processedScanReady(requestId, cloud)`, and `finished(requestId, result)`.

- [x] Add the tracker to the main window and invalidate it whenever CAD or scan data is replaced.
- [x] Pass the request id through the controller and reject stale UI callbacks.
- [x] Centralize action enablement so import and capture actions cannot change data during matching.
- [x] Review every successful input replacement and matching completion path for result invalidation.

### Task 3: Camera lifecycle and synchronization

**Files:**
- Modify: `dianyun/HikCameraWorker.h`
- Modify: `dianyun/HikCameraWorker.cpp`
- Modify: `dianyun/dianyun.cpp`

**Interfaces:**
- Produces: `isCapturing()` and `lastError()`; preserves `pointCloudReady` and `captureFinished`.

- [x] Add state-oriented checks for repeated start and invalid scan parameters.
- [x] Check every MVS return code and unwind partial initialization.
- [x] Protect the accumulated cloud and frame count with `QMutex`.
- [x] Move the completed cloud out under lock and emit signals after releasing the lock.

### Task 4: Matcher validation and obsolete-code removal

**Files:**
- Modify: `dianyun/MatchTypes.h`
- Modify: `dianyun/ModelMatchWorker.h`
- Modify: `dianyun/ModelMatchWorker.cpp`
- Modify: `dianyun/ModelMatchController.h`
- Modify: `dianyun/ModelMatchController.cpp`
- Modify: `dianyun/dianyun.cpp`

**Interfaces:**
- `ModelMatchController::start(cadCloud, scanCloud, config, requestId)` no longer consumes seams.
- `fineRegistrationB` consumes only the initial transform and output diagnostics.

- [x] Reject undersized post-filter clouds before coarse registration.
- [x] Guard the point-to-plane solve against singular systems and cap rotation and translation independently.
- [x] Validate the final transform as finite, homogeneous, orthonormal, and determinant-positive before inversion.
- [x] Remove unused seam weighting, seam projection, and their result/configuration fields.

### Task 5: Test project and documentation

**Files:**
- Create: `tests/dianyun.tests.vcxproj`
- Modify: `dianyun/dianyun.sln`
- Modify: `tests/WeldSeamMappingTests.cpp`
- Modify: `README.md`

**Interfaces:**
- Produces: a console test target included in Debug and Release solution builds.

- [x] Add mapping rejection cases for reflection, scale, non-finite coordinates, and empty geometry.
- [x] Mirror required Qt/PCL/OpenCascade build settings in the test project.
- [x] Update README functionality, mapping flow, concurrency safeguards, and file ownership.
- [x] Run `git diff --check`, test build, Release application build, and inspect the complete diff.
