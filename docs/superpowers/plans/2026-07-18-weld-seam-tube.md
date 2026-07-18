# Weld Seam Tube Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace thin screen-space CAD seam lines with visible model-space green tubes without changing seam centerline data.

**Architecture:** `WeldSeamData` remains the authoritative centerline. `VisualizationManager` converts valid centerline segments to one VTK polydata tube actor per seam and removes that actor by a stable ID.

**Tech Stack:** C++17, PCL Visualizer, VTK 9.3, Qt 6, Visual Studio 2022/MSBuild.

**Working Directory:** Run every command from the repository root `E:\data document` unless a command explicitly uses `Push-Location`.

## Global Constraints

- Tube diameter is `0.5%` of the current CAD bounding-box diagonal.
- Tubes use normal depth testing and do not show through the CAD model.
- Centerline points, segments, length, registration data, and future mapping inputs remain unchanged.
- This change does not implement CAD-to-scan seam mapping.

---

### Task 1: Replace seam lines with one VTK tube actor

**Files:**
- Modify: `dianyun/dianyun/VisualizationManager.h`
- Modify: `dianyun/dianyun/VisualizationManager.cpp`

**Interfaces:**
- Consumes: `void VisualizationManager::showSeam(const WeldSeamData& seam)` and `void VisualizationManager::removeSeam(const WeldSeamData& seam)`.
- Produces: one shape actor named `seam_tube_<seam.id>` and no changes to `WeldSeamData`.

- [x] **Step 1: Run a failing source-structure check**

Run:

```powershell
$source = Get-Content -Raw dianyun\dianyun\VisualizationManager.cpp
if ($source -notmatch 'vtkTubeFilter' -or $source -match 'addLine\(start, end') { exit 1 }
```

Expected: exit code `1`, because the current implementation uses `addLine` and has no `vtkTubeFilter`.

- [x] **Step 2: Add CAD scale state and stable actor naming**

Add to `VisualizationManager.h` private members:

```cpp
static std::string seamActorId(const WeldSeamData& seam);
double m_cadDiagonal = 0.0;
```

In `showCadModel`, store a finite positive `model.diagonal`; otherwise store `0.0`.

- [x] **Step 3: Build the VTK tube from valid centerline segments**

Include `vtkActor.h`, `vtkCellArray.h`, `vtkPoints.h`, `vtkPolyData.h`, `vtkPolyLine.h`, `vtkProperty.h`, `vtkSmartPointer.h`, and `vtkTubeFilter.h`.

In `showSeam`:

```cpp
auto points = vtkSmartPointer<vtkPoints>::New();
auto lines = vtkSmartPointer<vtkCellArray>::New();
Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
    std::numeric_limits<float>::infinity());
Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
    -std::numeric_limits<float>::infinity());
std::vector<WeldSeamSegment> validSegments;
validSegments.reserve(seam.segments.size());
for (const auto& segment : seam.segments) {
    if (!segment.start.allFinite() || !segment.end.allFinite() ||
        (segment.end - segment.start).squaredNorm() <= 1e-12f) {
        continue;
    }
    validSegments.push_back(segment);
    minimum = minimum.cwiseMin(segment.start).cwiseMin(segment.end);
    maximum = maximum.cwiseMax(segment.start).cwiseMax(segment.end);
}
if (validSegments.empty()) return;

double scale = m_cadDiagonal;
if (!std::isfinite(scale) || scale <= 0.0) {
    scale = static_cast<double>((maximum - minimum).norm());
}
if (!std::isfinite(scale) || scale <= 0.0) {
    scale = std::max(seam.length, 1.0);
}
const float connectionTolerance = static_cast<float>(
    std::max(scale * 1e-5, 1e-6));
const float connectionToleranceSquared =
    connectionTolerance * connectionTolerance;

const auto appendPath = [&](const std::vector<Eigen::Vector3f>& path) {
    auto polyline = vtkSmartPointer<vtkPolyLine>::New();
    polyline->GetPointIds()->SetNumberOfIds(
        static_cast<vtkIdType>(path.size()));
    for (size_t index = 0; index < path.size(); ++index) {
        const vtkIdType pointId = points->InsertNextPoint(
            path[index].x(), path[index].y(), path[index].z());
        polyline->GetPointIds()->SetId(
            static_cast<vtkIdType>(index), pointId);
    }
    lines->InsertNextCell(polyline);
};

std::vector<bool> used(validSegments.size(), false);
for (size_t seed = 0; seed < validSegments.size(); ++seed) {
    if (used[seed]) continue;
    used[seed] = true;
    std::vector<Eigen::Vector3f> path = {
        validSegments[seed].start, validSegments[seed].end
    };
    bool extended = true;
    while (extended) {
        extended = false;
        for (size_t index = 0; index < validSegments.size(); ++index) {
            if (used[index]) continue;
            const auto& segment = validSegments[index];
            if ((path.back() - segment.start).squaredNorm() <=
                connectionToleranceSquared) {
                path.push_back(segment.end);
            }
            else if ((path.back() - segment.end).squaredNorm() <=
                connectionToleranceSquared) {
                path.push_back(segment.start);
            }
            else if ((path.front() - segment.end).squaredNorm() <=
                connectionToleranceSquared) {
                path.insert(path.begin(), segment.start);
            }
            else if ((path.front() - segment.start).squaredNorm() <=
                connectionToleranceSquared) {
                path.insert(path.begin(), segment.end);
            }
            else {
                continue;
            }
            used[index] = true;
            extended = true;
            break;
        }
    }
    appendPath(path);
}

auto centerline = vtkSmartPointer<vtkPolyData>::New();
centerline->SetPoints(points);
centerline->SetLines(lines);

auto tube = vtkSmartPointer<vtkTubeFilter>::New();
tube->SetInputData(centerline);
tube->SetRadius(std::max(scale * 0.0025, 1e-6));
tube->SetNumberOfSides(12);
tube->CappingOn();
tube->Update();
```

Remove any existing actor with the same ID, add `tube->GetOutput()` through `addModelFromPolyData`, set color `(0.1, 1.0, 0.1)`, opacity `1.0`, pickable `false`, ambient `0.65`, and diffuse `0.35`.

- [x] **Step 4: Delete the tube actor as one unit**

Implement stable naming and deletion:

```cpp
std::string VisualizationManager::seamActorId(const WeldSeamData& seam)
{
    return "seam_tube_" + seam.id;
}

void VisualizationManager::removeSeam(const WeldSeamData& seam)
{
    if (!m_cadViewer) return;
    m_cadViewer->removeShape(seamActorId(seam));
    m_cadViewer->spinOnce(20, true);
}
```

- [x] **Step 5: Run the source-structure check again**

Run the command from Step 1.

Expected: exit code `0`.

- [x] **Step 6: Build Release x64**

Run:

```powershell
Push-Location dianyun\dianyun
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe' dianyun.vcxproj /t:Build /p:Configuration=Release /p:Platform=x64 /p:QtInstall=E:\Qt\6.10.1\msvc2022_64 /p:QtMsBuild=C:\Users\Z\AppData\Local\QtMsBuild /p:IncludePath=E:\vcpkg-master\installed\x64-windows\include
Pop-Location
```

Expected: build succeeds with `0` errors.

- [x] **Step 7: Run startup check**

Start `dianyun/dianyun/x64/Release/dianyun.exe`, verify it remains running for at least five seconds, then close the test process.

- [ ] **Step 8: Run manual visual checks**

In Visual Studio, import the existing STEP model, create and delete a seam, and verify tube width, depth occlusion, continuity, and deletion behavior.

- [ ] **Step 9: Commit the implementation**

```powershell
git add dianyun/dianyun/VisualizationManager.h dianyun/dianyun/VisualizationManager.cpp docs/superpowers/plans/2026-07-18-weld-seam-tube.md
git commit -m "改进焊缝三维管道显示"
```
