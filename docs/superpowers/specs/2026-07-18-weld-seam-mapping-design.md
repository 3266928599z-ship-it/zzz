# Weld Seam Mapping Design

## Goal

Add a `焊缝映射` action that maps the currently selected CAD weld seam into the scan coordinate system after a trusted model match.

## Confirmed Behavior

- Map only the currently selected seam.
- Enable mapping only when the latest match succeeded with `MatchVerdict::Pass`.
- Use the CAD-to-scan rigid transform `MatchResult::T_final`.
- Do not snap mapped points to nearest scan points.
- Keep the original seam green in the CAD viewer.
- Render the mapped seam as a yellow/orange tube in the scan viewer.
- Repeating the action replaces the prior mapped actor for that seam.

## Responsibilities

`WeldSeamService` validates the rigid matrix and transforms every seam point and segment endpoint. It has no UI or rendering responsibility.

`VisualizationManager` owns mapped seam actors in the scan viewer and provides show, remove, and clear operations.

`dianyun` coordinates selection, match-state validation, mapped result storage, button enablement, logging, and lifecycle cleanup.

## State And Lifecycle

Mapped seams are stored by source seam ID. Importing a CAD model, importing or capturing a scan, or starting a new match invalidates all mapped results. Deleting a CAD seam also removes its mapped result.

List operations use the seam ID stored in `Qt::UserRole`, rather than the row number, so internal list reordering cannot map or delete the wrong seam.

## Validation

Mapping rejects empty seams, non-finite matrices, non-homogeneous transforms, and transforms whose rotation block is not a proper near-orthonormal rotation. The mapped seam retains its source ID, name, and physical length.

## Future Extension

Nearest-scan distance, coverage, and missing-region statistics may be added as non-destructive quality indicators. They must not alter the rigidly mapped trajectory.
