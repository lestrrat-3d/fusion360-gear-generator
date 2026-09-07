#!/usr/bin/env python3
"""Verify the dated contract-rule classification against production manifests."""

import hashlib
import json
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
INVENTORY = HERE / "classification.json"
CLASSIFICATIONS = {
    "interface",
    "behavior",
    "intentional_implementation_structure",
}
TOP_LEVEL_FIELDS = {"_comment", "module", "module_constants", "classes", "source_guards"}
CLASS_FIELDS = {"bases", "methods", "ctx_fields"}
GUARD_FIELDS = {"file", "in_function", "why", "required", "banned"}


def escape_pointer(value):
    return str(value).replace("~", "~0").replace("/", "~1")


def pointer(*parts):
    return "/" + "/".join(escape_pointer(part) for part in parts)


def enforced_pointers(manifest):
    """Return the manifest entries that check_contract.py enforces."""
    result = set()
    for name in manifest.get("module_constants", {}):
        result.add(pointer("module_constants", name))
    for class_name, declaration in manifest.get("classes", {}).items():
        result.add(pointer("classes", class_name))
        for field in ("bases", "methods", "ctx_fields"):
            for index, _value in enumerate(declaration.get(field, [])):
                result.add(pointer("classes", class_name, field, index))
    for guard_index, guard in enumerate(manifest.get("source_guards", [])):
        result.add(pointer("source_guards", guard_index, "file"))
        if "in_function" in guard:
            result.add(pointer("source_guards", guard_index, "in_function"))
        for field in ("required", "banned"):
            for index, _value in enumerate(guard.get(field, [])):
                result.add(pointer("source_guards", guard_index, field, index))
    return result


def metadata_pointers(manifest):
    """Return non-rule leaf entries consumed only as documentation or routing data."""
    result = set()
    if "_comment" in manifest:
        result.add(pointer("_comment"))
    if "module" in manifest:
        result.add(pointer("module"))
    for guard_index, guard in enumerate(manifest.get("source_guards", [])):
        if "why" in guard:
            result.add(pointer("source_guards", guard_index, "why"))
    return result


def resolve_selector(document, selector):
    """Expand a JSON pointer whose ``*`` segments match one exact level."""
    if not selector.startswith("/"):
        raise ValueError(f"selector is not an absolute JSON pointer: {selector!r}")
    segments = selector[1:].split("/") if selector != "/" else []
    states = [(document, [])]
    for raw_segment in segments:
        segment = raw_segment.replace("~1", "/").replace("~0", "~")
        next_states = []
        for value, path in states:
            if segment == "*":
                if isinstance(value, dict):
                    next_states.extend((child, path + [key]) for key, child in value.items())
                elif isinstance(value, list):
                    next_states.extend((child, path + [index]) for index, child in enumerate(value))
                continue
            if isinstance(value, dict) and segment in value:
                next_states.append((value[segment], path + [segment]))
            elif isinstance(value, list) and segment.isdigit() and int(segment) < len(value):
                index = int(segment)
                next_states.append((value[index], path + [index]))
        states = next_states
    return {pointer(*path) for _value, path in states}


def duplicate_values(values):
    seen = set()
    duplicates = set()
    for value in values:
        if value in seen:
            duplicates.add(value)
        seen.add(value)
    return duplicates


def unknown_field_errors(path, manifest):
    """Reject manifest fields whose checker meaning is not part of this review."""
    errors = []
    for field in sorted(set(manifest) - TOP_LEVEL_FIELDS):
        errors.append(f"{path}: unknown top-level manifest field: {pointer(field)}")
    for class_name, declaration in manifest.get("classes", {}).items():
        for field in sorted(set(declaration) - CLASS_FIELDS):
            errors.append(
                f"{path}: unknown class manifest field: {pointer('classes', class_name, field)}"
            )
    for guard_index, guard in enumerate(manifest.get("source_guards", [])):
        for field in sorted(set(guard) - GUARD_FIELDS):
            errors.append(
                f"{path}: unknown guard manifest field: "
                f"{pointer('source_guards', guard_index, field)}"
            )
    return errors


def check_manifest(entry):
    path = ROOT / entry["path"]
    raw = path.read_bytes()
    manifest = json.loads(raw)
    errors = unknown_field_errors(entry["path"], manifest)
    digest = hashlib.sha256(raw).hexdigest()
    if digest != entry["sha256"]:
        errors.append(f"{entry['path']}: SHA-256 changed: {digest}")

    classified = []
    class_counts = {name: 0 for name in sorted(CLASSIFICATIONS)}
    group_ids = [group["id"] for group in entry["groups"]]
    for duplicate in sorted(duplicate_values(group_ids)):
        errors.append(f"{entry['path']}: duplicate group id {duplicate!r}")
    for group in entry["groups"]:
        classification = group["classification"]
        if classification not in CLASSIFICATIONS:
            errors.append(
                f"{entry['path']}: group {group['id']!r} has unknown classification "
                f"{classification!r}"
            )
            continue
        members = set()
        for selector in group["selectors"]:
            resolved = resolve_selector(manifest, selector)
            if not resolved:
                errors.append(
                    f"{entry['path']}: group {group['id']!r} selector {selector!r} matches nothing"
                )
            members.update(resolved)
        classified.extend(members)
        class_counts[classification] += len(members)

    expected = enforced_pointers(manifest)
    classified_set = set(classified)
    for duplicate in sorted(duplicate_values(classified)):
        errors.append(f"{entry['path']}: rule classified more than once: {duplicate}")
    for missing in sorted(expected - classified_set):
        errors.append(f"{entry['path']}: enforced rule is unclassified: {missing}")
    for unknown in sorted(classified_set - expected):
        errors.append(f"{entry['path']}: classified pointer is not an enforced rule: {unknown}")

    documented_metadata = []
    for metadata in entry["metadata"]:
        resolved = resolve_selector(manifest, metadata["selector"])
        if not resolved:
            errors.append(
                f"{entry['path']}: metadata selector {metadata['selector']!r} matches nothing"
            )
        documented_metadata.extend(resolved)
    expected_metadata = metadata_pointers(manifest)
    documented_set = set(documented_metadata)
    for duplicate in sorted(duplicate_values(documented_metadata)):
        errors.append(f"{entry['path']}: metadata classified more than once: {duplicate}")
    for missing in sorted(expected_metadata - documented_set):
        errors.append(f"{entry['path']}: metadata is undocumented: {missing}")
    for unknown in sorted(documented_set - expected_metadata):
        errors.append(f"{entry['path']}: documented metadata pointer is unknown: {unknown}")

    return errors, len(expected), class_counts, len(expected_metadata)


def main():
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    errors = []
    total_rules = 0
    total_metadata = 0
    total_classes = {name: 0 for name in sorted(CLASSIFICATIONS)}
    for entry in inventory["manifests"]:
        current_errors, rule_count, class_counts, metadata_count = check_manifest(entry)
        errors.extend(current_errors)
        total_rules += rule_count
        total_metadata += metadata_count
        for name, count in class_counts.items():
            total_classes[name] += count
        print(
            f"{entry['gear']}: {rule_count} enforced rules, {metadata_count} metadata entries, "
            + ", ".join(f"{name}={count}" for name, count in class_counts.items())
        )
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"total: {total_rules} enforced rules, {total_metadata} metadata entries, "
        + ", ".join(f"{name}={count}" for name, count in total_classes.items())
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
