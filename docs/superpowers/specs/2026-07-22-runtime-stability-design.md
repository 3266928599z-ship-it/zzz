# Runtime Stability Design

## Goal

Prevent stale asynchronous results, harden the camera lifecycle, reject degenerate matching inputs, and remove obsolete seam-matching code without changing the validated registration thresholds or rigid seam-mapping behavior.

## Scope

- Tag every match request and discard progress, preview, or final callbacks that do not belong to the current input revision.
- Disable data-changing actions while matching and keep revision checks as a second line of defense.
- Check every Hikvision SDK transition, clean up partial initialization, and serialize access to accumulated camera data.
- Validate downsampled point clouds and final rigid transforms before entering later registration stages.
- Remove unused seam-weighted ICP and legacy seam-projection fields. Seam mapping remains owned by `WeldSeamService`.
- Add an executable test project to the Visual Studio solution and update the README.

## Non-Goals

- Do not retune registration thresholds.
- Do not add nearest-neighbor seam snapping or local seam deformation.
- Do not introduce a new independent validation dataset until labelled registration samples are available.
- Do not require camera hardware for automated tests.

## Design

`MatchRequestTracker` owns a monotonically increasing data revision and request id. Replacing CAD or scan data invalidates the active request. `ModelMatchController` attaches the request id to all worker notifications. The main window checks the id before displaying progress, preview data, or a final transform.

`HikCameraWorker` treats an SDK handle as valid only after create, open, callback registration, and stream startup all succeed. Failed initialization unwinds in reverse order. A mutex protects the accumulated cloud and frame counter; the callback transfers ownership of a completed cloud before emitting Qt signals.

`ModelMatchWorker` validates post-filter point counts, finite scale values, linear-system conditioning, and the final proper-rigid transform. Legacy seam weighting and projection are removed because the active pipeline does not use them.

## Acceptance Criteria

- A result created for old CAD or scan data cannot become the current result.
- Repeated camera-start requests cannot clear a cloud while a callback writes to it.
- Every camera initialization failure leaves a null handle and inactive capture state.
- Degenerate post-filter clouds fail with a diagnostic before PPF, PCA, or ICP.
- Model matching no longer accepts seam data or reports unused seam weights.
- The test executable covers request invalidation and rigid-transform rejection.
