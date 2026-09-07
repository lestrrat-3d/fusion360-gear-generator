"""Record Fusion's native behavior for the spur anchor projection chain."""

import datetime
import json
import os
import traceback

import adsk.core
import adsk.fusion


PROBE_NAME = "spur_projection_constraint_chain"
OUTPUT_PREFIX = "projection_probe_results_"


def _utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _error_record(error):
    return {
        "type": type(error).__name__,
        "message": str(error),
        "traceback": traceback.format_exc(),
    }


def _property_state(obj, name):
    try:
        value = getattr(obj, name)
    except AttributeError as error:
        return {"status": "unavailable", "error": str(error)}
    except Exception as error:
        return {"status": "error", "error": "%s: %s" % (type(error).__name__, error)}
    if isinstance(value, (bool, float, int, str)) or value is None:
        return {"status": "available", "value": value}
    return {"status": "available", "value": str(value)}


def _point_property_state(point, name):
    try:
        value = getattr(point, name)
    except AttributeError as error:
        return {"status": "unavailable", "error": str(error)}
    except Exception as error:
        return {"status": "error", "error": "%s: %s" % (type(error).__name__, error)}
    try:
        return {
            "status": "available",
            "value": {"x": float(value.x), "y": float(value.y), "z": float(value.z)},
        }
    except Exception as error:
        return {"status": "error", "error": "%s: %s" % (type(error).__name__, error)}


def _entity_state(entity):
    return {
        "object_type": _property_state(entity, "objectType"),
        "is_fully_constrained": _property_state(entity, "isFullyConstrained"),
        "is_fixed": _property_state(entity, "isFixed"),
        "is_reference": _property_state(entity, "isReference"),
        "is_linked": _property_state(entity, "isLinked"),
    }


def _sketch_point_state(point):
    result = _entity_state(point)
    result["sketch_position_cm"] = _point_property_state(point, "geometry")
    result["world_position_cm"] = _point_property_state(point, "worldGeometry")
    return result


def _sketch_state(sketch):
    return {
        "name": _property_state(sketch, "name"),
        "parent_component": _property_state(sketch.parentComponent, "name"),
        "is_fully_constrained": _property_state(sketch, "isFullyConstrained"),
    }


def _active_component_state(design):
    try:
        component = design.activeComponent
    except AttributeError as error:
        return {"status": "unavailable", "error": str(error)}
    except Exception as error:
        return {"status": "error", "error": "%s: %s" % (type(error).__name__, error)}
    if component is None:
        return {"status": "available", "value": None}
    return {"status": "available", "value": _property_state(component, "name")}


def _circle_state(circle):
    result = _entity_state(circle)
    result["center"] = _sketch_point_state(circle.centerSketchPoint)
    result["radius_cm"] = _property_state(circle, "radius")
    return result


def _write_report(report, output_path):
    report["last_updated_utc"] = _utc_now()
    temporary_path = output_path + ".tmp"
    with open(temporary_path, "w", encoding="utf-8") as output_file:
        json.dump(report, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    os.replace(temporary_path, output_path)


def _run_stage(report, output_path, name, operation):
    stage = {
        "name": name,
        "status": "running",
        "started_utc": _utc_now(),
        "observations": {},
    }
    report["stages"].append(stage)
    _write_report(report, output_path)
    try:
        operation(stage["observations"])
    except Exception as error:
        stage["status"] = "error"
        stage["completed_utc"] = _utc_now()
        stage["error"] = _error_record(error)
        _write_report(report, output_path)
        raise
    stage["status"] = "complete"
    stage["completed_utc"] = _utc_now()
    _write_report(report, output_path)


def _require(value, message):
    if value is None:
        raise RuntimeError(message)
    return value


def _document_identity(document):
    return {
        "creation_id": _property_state(document, "creationId"),
        "is_saved": _property_state(document, "isSaved"),
        "is_valid": _property_state(document, "isValid"),
        "name": _property_state(document, "name"),
    }


def _documents_match(left, right):
    if left is None or right is None:
        return False
    if left is right:
        return True
    try:
        if left == right:
            return True
    except Exception:
        pass
    try:
        left_id = left.creationId
        right_id = right.creationId
        return bool(left_id) and left_id == right_id
    except Exception:
        return False


def _set_fixed(point, expected, observations, key):
    setattr(point, "isFixed", expected)
    state = _property_state(point, "isFixed")
    observations[key] = state
    if state.get("status") != "available" or state.get("value") is not expected:
        raise RuntimeError("SketchPoint.isFixed did not become %s" % expected)


def _compute_all(design, observations, key):
    completed = design.computeAll()
    observations[key] = completed
    if completed is not True:
        raise RuntimeError("Design.computeAll returned %r" % completed)


def _project_one(sketch, source, observations):
    call = {
        "expression": "sketch.project(entity)",
        "fallback_used": False,
        "status": "attempting",
    }
    observations["projection_call"] = call
    try:
        project = getattr(sketch, "project")
    except AttributeError as error:
        call["status"] = "unavailable"
        call["error"] = str(error)
        raise RuntimeError("legacy Sketch.project is unavailable; project2 was not substituted") from error
    call["status"] = "available"
    try:
        projected = project(source)
    except Exception as error:
        call["status"] = "execution_error"
        call["error"] = "%s: %s" % (type(error).__name__, error)
        raise
    call["status"] = "returned"
    if projected is None:
        call["return_count"] = None
        raise RuntimeError("Sketch.project returned None")
    try:
        count = projected.count
        call["return_count"] = count
        if count != 1:
            raise RuntimeError("Sketch.project returned %d entities; expected one point" % count)
        result = projected.item(0)
    except AttributeError as error:
        call["collection_status"] = "unexpected_return_shape"
        call["error"] = str(error)
        raise RuntimeError("Sketch.project did not return the documented ObjectCollection shape") from error
    call["collection_status"] = "accepted"
    return _require(result, "Sketch.project returned a null entity")


def _world_position(point_state):
    position = point_state.get("world_position_cm", {})
    if position.get("status") != "available":
        return None
    return position.get("value")


def _delta(before, after):
    before_position = _world_position(before)
    after_position = _world_position(after)
    if before_position is None or after_position is None:
        return None
    return {
        axis: after_position[axis] - before_position[axis]
        for axis in ("x", "y", "z")
    }


def _chain_snapshot(runtime):
    result = {
        "source": _sketch_point_state(runtime["source_point"]),
        "tools_projection": _sketch_point_state(runtime["tools_projection"]),
        "bore_projection": _sketch_point_state(runtime["bore_projection"]),
        "bore_local_origin": _sketch_point_state(runtime["bore_local_origin"]),
        "bore_circle_center": _sketch_point_state(runtime["bore_circle"].centerSketchPoint),
        "source_sketch": _sketch_state(runtime["source_sketch"]),
        "tools_sketch": _sketch_state(runtime["tools_sketch"]),
        "bore_sketch": _sketch_state(runtime["bore_sketch"]),
    }
    if "repair_projection" in runtime:
        result["repair_projection"] = _sketch_point_state(runtime["repair_projection"])
        result["repair_fixed_anchor"] = _sketch_point_state(runtime["repair_fixed_anchor"])
        result["repair_sketch"] = _sketch_state(runtime["repair_sketch"])
    return result


def _snapshot_deltas(before, after):
    result = {}
    for key in sorted(set(before).intersection(after)):
        if "world_position_cm" in before[key] and "world_position_cm" in after[key]:
            result[key] = _delta(before[key], after[key])
    return result


def _move_source(runtime, vector, observations, require_success):
    before = _chain_snapshot(runtime)
    observations["before"] = before
    _set_fixed(runtime["source_point"], False, observations, "source_unfixed_readback")
    move_error = None
    move_return = None
    try:
        move_return = runtime["source_point"].move(vector)
        observations["move_return"] = move_return
    except Exception as error:
        move_error = error
        observations["move_error"] = "%s: %s" % (type(error).__name__, error)
    finally:
        _set_fixed(runtime["source_point"], True, observations, "source_refixed_readback")
    _compute_all(runtime["design"], observations, "compute_all_return")
    after = _chain_snapshot(runtime)
    observations["after"] = after
    observations["observed_world_deltas_cm"] = _snapshot_deltas(before, after)
    if move_error is not None:
        raise move_error
    if require_success and move_return is not True:
        raise RuntimeError("SketchPoint.move returned %r during the baseline tracking check" % move_return)


def run(context):
    del context
    output_name = OUTPUT_PREFIX + datetime.datetime.now().strftime("%Y%m%dT%H%M%S_%f") + ".json"
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), output_name)
    report = {
        "schema": 1,
        "probe": PROBE_NAME,
        "native_run": {"status": "running", "started_utc": _utc_now()},
        "projection_api": {
            "required_call": "Sketch.project(entity)",
            "project2_substitution_allowed": False,
        },
        "fixture": {
            "source": "fixed point in a root-component XY sketch",
            "target": "Tools and Bore sketches on the XY plane of one child component",
            "bore_radius_cm": 1.0,
        },
        "stages": [],
    }
    runtime = {}
    ui = None
    _write_report(report, output_path)

    try:
        app = _require(adsk.core.Application.get(), "Application.get returned None")
        ui = app.userInterface
        report["fusion_version"] = _property_state(app, "version")
        _write_report(report, output_path)

        def create_owned_design(observations):
            created_document = _require(
                app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType),
                "Documents.add returned None",
            )
            active_document = app.activeDocument
            observations["created_document"] = _document_identity(created_document)
            observations["active_document"] = (
                _document_identity(active_document) if active_document is not None else None
            )
            observations["active_matches_created"] = _documents_match(
                active_document, created_document
            )
            if not observations["active_matches_created"]:
                raise RuntimeError(
                    "the newly created document is not active; refusing to inspect activeProduct"
                )
            if created_document.isSaved:
                raise RuntimeError("the newly created probe document unexpectedly reports as saved")
            created_document.name = "Projection Constraint Probe (unsaved)"
            design = _require(
                adsk.fusion.Design.cast(app.activeProduct),
                "the owned document's active product is not a Fusion Design",
            )
            parent_document = design.parentDocument
            observations["design_parent_document"] = _document_identity(parent_document)
            observations["design_parent_matches_created"] = _documents_match(
                parent_document, created_document
            )
            if not observations["design_parent_matches_created"]:
                raise RuntimeError("Design.parentDocument does not match the owned probe document")
            runtime["document"] = created_document
            runtime["design"] = design
            runtime["root_component"] = design.rootComponent
            observations["root_component_name"] = _property_state(
                runtime["root_component"], "name"
            )

        _run_stage(report, output_path, "create_and_verify_owned_design", create_owned_design)

        def create_fixture_context(observations):
            root = runtime["root_component"]
            source_sketch = _require(
                root.sketches.add(root.xYConstructionPlane),
                "failed to create the root source sketch",
            )
            source_sketch.name = "Projection Probe Source"
            source_point = _require(
                source_sketch.sketchPoints.add(adsk.core.Point3D.create(2.0, 3.0, 0.0)),
                "failed to create the source point",
            )
            _set_fixed(source_point, True, observations, "source_fixed_readback")
            occurrence = _require(
                root.occurrences.addNewComponent(adsk.core.Matrix3D.create()),
                "failed to create the child component occurrence",
            )
            child = _require(occurrence.component, "the new occurrence has no component")
            child.name = "Projection Probe Component"
            runtime.update(
                {
                    "source_sketch": source_sketch,
                    "source_point": source_point,
                    "child_occurrence": occurrence,
                    "child_component": child,
                }
            )
            _compute_all(runtime["design"], observations, "compute_all_return")
            observations["component_context"] = {
                "source_component": _property_state(source_sketch.parentComponent, "name"),
                "target_component": _property_state(child, "name"),
                "design_active_component": _active_component_state(runtime["design"]),
                "source_plane": "rootComponent.xYConstructionPlane",
                "target_plane": "childComponent.xYConstructionPlane",
            }
            observations["source_point"] = _sketch_point_state(source_point)
            observations["source_sketch"] = _sketch_state(source_sketch)

        _run_stage(report, output_path, "create_root_source_and_child_context", create_fixture_context)

        def project_to_tools(observations):
            child = runtime["child_component"]
            tools_sketch = _require(
                child.sketches.add(child.xYConstructionPlane),
                "failed to create the Tools sketch",
            )
            tools_sketch.name = "Projection Probe Tools"
            tools_projection = _project_one(
                tools_sketch, runtime["source_point"], observations
            )
            runtime["tools_sketch"] = tools_sketch
            runtime["tools_projection"] = tools_projection
            _compute_all(runtime["design"], observations, "compute_all_return")
            observations["design_active_component"] = _active_component_state(runtime["design"])
            observations["projected_point"] = _sketch_point_state(tools_projection)
            observations["reference_only_sketch"] = _sketch_state(tools_sketch)

        _run_stage(report, output_path, "project_source_to_reference_only_tools", project_to_tools)

        def build_bore_recipe(observations):
            child = runtime["child_component"]
            bore_sketch = _require(
                child.sketches.add(child.xYConstructionPlane),
                "failed to create the Bore sketch",
            )
            bore_sketch.name = "Projection Probe Bore"
            local_origin = _require(
                bore_sketch.sketchPoints.add(adsk.core.Point3D.create(0.0, 0.0, 0.0)),
                "failed to create the movable local-origin point",
            )
            bore_projection = _project_one(
                bore_sketch, runtime["tools_projection"], observations
            )
            circle = _require(
                bore_sketch.sketchCurves.sketchCircles.addByCenterRadius(
                    bore_projection, report["fixture"]["bore_radius_cm"]
                ),
                "failed to create the bore circle",
            )
            dimension = _require(
                bore_sketch.sketchDimensions.addDiameterDimension(
                    circle, adsk.core.Point3D.create(2.0, 4.0, 0.0)
                ),
                "failed to create the driving diameter dimension",
            )
            runtime.update(
                {
                    "bore_sketch": bore_sketch,
                    "bore_projection": bore_projection,
                    "bore_local_origin": local_origin,
                    "bore_circle": circle,
                    "bore_dimension": dimension,
                }
            )
            _compute_all(runtime["design"], observations, "compute_before_coincidence_return")
            source_fixed = _property_state(runtime["source_point"], "isFixed")
            dimension_driving = _property_state(dimension, "isDriving")
            local_origin_state = _sketch_point_state(local_origin)
            observations["before_coincidence"] = {
                "design_active_component": _active_component_state(runtime["design"]),
                "source_point_is_fixed": source_fixed,
                "sketch": _sketch_state(bore_sketch),
                "projected_anchor": _sketch_point_state(bore_projection),
                "movable_local_origin": local_origin_state,
                "circle": _circle_state(circle),
                "diameter_dimension_is_driving": dimension_driving,
            }
            if source_fixed.get("status") != "available" or source_fixed.get("value") is not True:
                raise RuntimeError("the source point is not verified fixed; Bore result is invalid")
            if (
                dimension_driving.get("status") != "available"
                or dimension_driving.get("value") is not True
            ):
                raise RuntimeError("the bore diameter dimension is not verified driving")
            local_constraint = local_origin_state["is_fully_constrained"]
            if (
                local_constraint.get("status") != "available"
                or local_constraint.get("value") is not False
            ):
                raise RuntimeError(
                    "the local origin is not verified free before coincidence; Bore result is invalid"
                )
            coincidence = _require(
                bore_sketch.geometricConstraints.addCoincident(
                    local_origin, bore_projection
                ),
                "addCoincident returned None for the Bore local origin and projected anchor",
            )
            runtime["bore_coincidence"] = coincidence
            _compute_all(runtime["design"], observations, "compute_after_coincidence_return")
            observations["after_coincidence"] = {
                "design_active_component": _active_component_state(runtime["design"]),
                "sketch": _sketch_state(bore_sketch),
                "projected_anchor": _sketch_point_state(bore_projection),
                "movable_local_origin": _sketch_point_state(local_origin),
                "circle": _circle_state(circle),
                "diameter_dimension_is_driving": _property_state(dimension, "isDriving"),
                "coincidence_is_valid": _property_state(coincidence, "isValid"),
            }

        _run_stage(report, output_path, "build_bore_recipe", build_bore_recipe)

        def baseline_tracking(observations):
            _move_source(
                runtime,
                adsk.core.Vector3D.create(0.5, 0.25, 0.0),
                observations,
                require_success=True,
            )

        _run_stage(report, output_path, "move_source_before_fixed_anchor_repair", baseline_tracking)

        def add_fixed_anchor_repair(observations):
            child = runtime["child_component"]
            repair_sketch = _require(
                child.sketches.add(child.xYConstructionPlane),
                "failed to create the repair-check sketch",
            )
            repair_sketch.name = "Projection Probe Fixed Anchor Repair"
            repair_projection = _project_one(
                repair_sketch, runtime["tools_projection"], observations
            )
            position = repair_projection.geometry
            fixed_anchor = _require(
                repair_sketch.sketchPoints.add(
                    adsk.core.Point3D.create(position.x, position.y, position.z)
                ),
                "failed to create the repair's local anchor point",
            )
            _set_fixed(fixed_anchor, True, observations, "repair_anchor_fixed_readback")
            coincidence = _require(
                repair_sketch.geometricConstraints.addCoincident(
                    repair_projection, fixed_anchor
                ),
                "addCoincident returned None for the projection and fixed repair anchor",
            )
            runtime.update(
                {
                    "repair_sketch": repair_sketch,
                    "repair_projection": repair_projection,
                    "repair_fixed_anchor": fixed_anchor,
                    "repair_coincidence": coincidence,
                }
            )
            _compute_all(runtime["design"], observations, "compute_all_return")
            observations["design_active_component"] = _active_component_state(runtime["design"])
            observations["repair_sketch"] = _sketch_state(repair_sketch)
            observations["projected_anchor"] = _sketch_point_state(repair_projection)
            observations["fixed_local_anchor"] = _sketch_point_state(fixed_anchor)
            observations["coincidence_is_valid"] = _property_state(coincidence, "isValid")

        _run_stage(report, output_path, "add_fixed_local_anchor_repair", add_fixed_anchor_repair)

        def tracking_after_repair(observations):
            _move_source(
                runtime,
                adsk.core.Vector3D.create(0.5, -0.25, 0.0),
                observations,
                require_success=False,
            )

        _run_stage(report, output_path, "move_source_after_fixed_anchor_repair", tracking_after_repair)

        report["native_run"]["status"] = "complete"
        report["native_run"]["completed_utc"] = _utc_now()
        _write_report(report, output_path)
        if ui is not None:
            ui.messageBox(
                "Projection probe complete.\n\nJSON: %s\n\n"
                "The unsaved probe design remains open for inspection." % output_path
            )
    except Exception as error:
        report["native_run"]["status"] = "error"
        report["native_run"]["completed_utc"] = _utc_now()
        report["fatal_error"] = _error_record(error)
        try:
            _write_report(report, output_path)
        except Exception as write_error:
            report["report_write_error"] = "%s: %s" % (
                type(write_error).__name__, write_error
            )
        if ui is not None:
            ui.messageBox(
                "Projection probe stopped. Earlier stage evidence was retained when possible.\n\n"
                "JSON: %s\n\n%s" % (output_path, traceback.format_exc())
            )
