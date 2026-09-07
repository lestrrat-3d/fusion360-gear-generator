# T5. Use one API-status decision

## Assignment

The implementer makes drafting and validation in `fusion360-gear-generator` use the same API-status rules.
T0 must be complete. The [common execution contract](README.md#common-execution-contract) applies.
The policy tables already in `fusion_api.py` remain the sole owners of runtime exceptions.

## Allowed edits and entry points

Allowed edits are `fusion_api.py`, `check_compile.py`, `check_api_calls.py`, their tests,
`test_fusion_api_session.py`, `test_render_prompt.py`, and the compile and emit prompts and skills.
The implementer may add `query_api_status.py` and `test_query_api_status.py`.
The implementer reads `member_info`, `class_members`, `lookup_many`, `unverified_class`,
`unverified_findings`, `REFUTED_CALLS`, and `UNVERIFIED_CALLS` before editing.
The API database, external plugin, policy-table entries, and production specs are read-only.

## Fixed interface

`fusion_api.py` adds `describe_call(owner: str | None, name: str) -> dict`.
It uses the existing query session and returns exactly these keys:

```json
{
  "schema": 1,
  "owner": "adsk.fusion.WidgetTools",
  "name": "addWidget",
  "status": "documented",
  "scope": "receiver",
  "disposition": "allow",
  "declared_on": "adsk.fusion.WidgetTools",
  "returns": null,
  "evidence": ["The database declares the requested member."],
  "stale_watchlist": false
}
```

`owner` is a qualified class or null. Invalid owner syntax and invalid member identifiers raise `ValueError`.
`scope` is `receiver` when an owner is supplied and `name_only` otherwise.
Database exceptions become `unavailable` results with the original reason in `evidence`.
`declared_on` and `returns` are null when the evidence cannot provide them.

## Decision table

Rows are evaluated in this order. They apply to an explicit owner unless the row says otherwise.

| Condition | Status | Disposition |
|---|---|---|
| Exact owner/member occurs in `REFUTED_CALLS` | `refuted` | `block` |
| Database lookup fails | `unavailable` | `setup_error` |
| `member_info` or inherited `class_members` resolves the member | `documented` | `allow` |
| Exact owner/member occurs in `UNVERIFIED_CALLS` but lacks database support | `unverified` | `advisory` |
| No matching declaration and no policy-table entry exists | `not_found` | `block` |
| Owner is null and `lookup(name)` returns at least one hit | `documented` with `name_only` scope | `allow` |
| Owner is null and no hit exists | `not_found` with `name_only` scope | `block` |

For owner-null queries, the final two rows replace the owner-specific database rows; they do not apply
receiver-specific policy exceptions by name alone. A `name_only` result establishes no receiver compatibility.
The emitted-code checker continues to reject unknown receivers before accepting a name-only result.
An explicit unknown receiver is not silently replaced by an unscoped lookup.

When a documented member still has a watchlist entry, `stale_watchlist` is true and `evidence` includes
the existing stale-entry advisory. The implementer does not remove the table entry automatically.
`not_found` describes database evidence; diagnostics must not call it a proven runtime absence.

## Ordered edits

1. The implementer adds the shared resolver and table-driven tests with mocked query responses.
2. `query_api_status.py --owner <class> --member <name>` emits one sorted JSON object.
   It exits 0 for allow/advisory, 1 for block, and 2 for invalid arguments or setup_error.
3. The compiler keeps its local/Python-name filtering. Legacy receiver deduction uses the existing helpers;
   when it cannot deduce an owner, it keeps legacy name-only behavior. Version 2 later supplies explicit owners.
4. The emitted-code checker passes inferred owner classes into the same resolver. Its unknown-receiver,
   reachable-call, and actual-member checks retain their current effect.
5. Both checkers render diagnostics from the result without maintaining separate policy tables.
   Existing callable lookup/session interfaces remain available to their other consumers.
6. Both prompts use the new CLI for status decisions. Signature detail still comes from the existing query tool.
   The emitter's claim that all compiled calls are verified is replaced with the actual documented/unverified policy.
7. The implementer retains query-session reuse and existing batching; this refactor must not launch one process
   per repeated occurrence of the same `(owner, name)` during a single check.

## Fixture matrix

| Test name | Fixture | Expected result |
|---|---|---|
| `test_documented_member` | The fake database declares `WidgetTools.addWidget`. | `documented`, `receiver`, `allow`. |
| `test_inherited_member` | The member is declared on a base class. | `declared_on` names that base. |
| `test_watchlist_entries` | Each current watchlist entry lacks database support. | `unverified`, `advisory`. |
| `test_wrong_owner_not_exempt` | A watchlist member is queried on an unrelated class. | `not_found`, `block`. |
| `test_refuted_precedence` | A refuted pair is also present in the database. | `refuted`, `block`. |
| `test_database_unavailable` | The transport raises `Unavailable`. | `unavailable`, `setup_error`; CLI exits 2. |
| `test_stale_watchlist` | A watchlist pair is now documented. | `documented`, plus the stale-entry advisory. |
| `test_name_only_is_not_typed` | Only a member name is supplied. | `scope=name_only`; emitted unknown receivers still fail. |
| `test_legacy_results_preserved` | Existing accepted step-call fixtures run. | Their pass/fail behavior is unchanged. |
| `test_shared_policy` | Both checkers receive the same typed pair. | Their status and severity agree. |
| `test_repeated_lookup_reuses_session` | A typed pair occurs repeatedly. | Existing session reuse and batching tests pass. |

## Validation and completion

```sh
python3 -m unittest discover -s "$P" -p 'test_fusion_api*.py' > .tmp/t5-api-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_query_api_status.py' > .tmp/t5-cli-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_check_api_calls.py' > .tmp/t5-receiver-tests.txt 2>&1
```

The [common validation](README.md#common-validation) also applies.
A required new runtime exception or an unexplained change in accepted legacy results is an escalation.
Completion requires shared status decisions, visible uncertainty, and preserved receiver rejection.
