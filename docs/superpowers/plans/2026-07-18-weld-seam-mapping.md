# Weld Seam Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Map the selected CAD weld seam into the scan coordinate system after a trusted model match and display it as a distinct tube in the scan viewer.

**Architecture:** `WeldSeamService` performs a validated rigid transformation. `VisualizationManager` owns the right-view actor, while `dianyun` owns action state, seam identity, mapped data, and lifecycle cleanup.

**Tech Stack:** C++17, Qt 6 Widgets, Eigen, PCL, VTK, Visual Studio/MSBuild

## Global Constraints

- Map only the selected seam.
- Require `MatchVerdict::Pass`.
- Use `MatchResult::T_final` without nearest-neighbor snapping.
- Preserve current uncommitted weld-tube display work.
- Do not add third-party dependencies.

---

### Task 1: Rigid Mapping Service

**Files:**
- Modify: `dianyun/dianyun/WeldSeamService.h`
- Modify: `dianyun/dianyun/WeldSeamService.cpp`
- Create: `dianyun/tests/WeldSeamMappingTests.cpp`

**Interfaces:**
- Consumes: `WeldSeamData`, `Eigen::Matrix4f`
- Produces: `bool WeldSeamService::mapSeam(const WeldSeamData&, const Eigen::Matrix4f&, WeldSeamData&, QString&) const`

- [ ] Write a test that creates one point and one segment, applies a 90-degree Z rotation plus translation, and checks transformed coordinates, identity, metadata, and length.
- [ ] Compile it before adding `mapSeam`; expect failure because the service has no member named `mapSeam`.
- [ ] Implement finite/homogeneous/proper-rigid matrix validation and homogeneous point/segment conversion.
- [ ] Run the mapping test; expect exit code 0.

### Task 2: Scan-View Mapped Tube

**Files:**
- Modify: `dianyun/dianyun/VisualizationManager.h`
- Modify: `dianyun/dianyun/VisualizationManager.cpp`

**Interfaces:**
- Produces: `showMappedSeam`, `removeMappedSeam`, `clearMappedSeams`

- [ ] Extract current tube construction into a private helper accepting viewer, actor ID, scale, and RGB values.
- [ ] Keep CAD seams green; add mapped seams to the scan viewer in yellow/orange as `mapped_seam_tube_<id>`.
- [ ] Track mapped actor IDs so clear removes only mapped seam actors.
- [ ] Build `Release|x64`; expect zero errors.

### Task 3: UI Action And State Coordination

**Files:**
- Modify: `dianyun/dianyun/dianyun.ui`
- Modify: `dianyun/dianyun/dianyun.h`
- Modify: `dianyun/dianyun/dianyun.cpp`

**Interfaces:**
- Consumes: `WeldSeamService::mapSeam`, `VisualizationManager::showMappedSeam`
- Produces: `on_btnMapSeam_clicked`, `updateMapSeamButtonState`, `clearMappedSeamResults`

- [ ] Add disabled `btnMapSeam` with text `焊缝映射`.
- [ ] Store each seam ID in its list item's `Qt::UserRole`.
- [ ] Resolve mapping and deletion by ID, not mutable list row.
- [ ] Enable only for a selected seam and the latest `success + Pass` match.
- [ ] Map, store/replace, display, and log the selected seam.
- [ ] Clear mapped state on CAD/scan replacement, camera scan replacement, match restart/failure, and source seam deletion.

### Task 4: Verification

- [ ] Run the mapping test and confirm exit code 0.
- [ ] Build `dianyun.sln` as `Release|x64` and confirm zero errors.
- [ ] Launch the generated executable briefly and confirm no immediate startup failure.
- [ ] Run `git diff --check` and inspect the scoped diff.
