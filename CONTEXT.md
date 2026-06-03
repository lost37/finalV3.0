# Smartcar Control

This context describes the smartcar's competition-control language, especially element handling around red-block recognition and avoidance.

## Language

**Red Block Perception**:
The source of trusted observations about a red block and its model region of interest.
_Avoid_: redblock detection state machine

**Red Block Decision**:
The state machine that decides when a red block encounter is confirmed and which high-level action should be executed.
_Avoid_: redblock all-in-one state machine

**Red Block Motion**:
The executor for physical actions requested by Red Block Decision, such as braking, low-speed approach, bypass, straight pass, and recovery.
_Avoid_: redblock decision

**Brake Gate**:
The completion point after reverse braking and a short low-speed hold that allows rolling recognition to begin.
_Avoid_: full stop requirement, strict encoder-speed gate

**Rolling Model Recognition**:
Model recognition performed while the car continues moving at a controlled low speed after passing the Brake Gate.
_Avoid_: parked recognition, stop-and-detect

**Low-Speed Hold**:
The motion mode that keeps the car at red-block recognition speed while allowing limited steering correction.
_Avoid_: ordinary speed decision

**Low-Speed Settle**:
The five-frame Low-Speed Hold period after reverse braking before rolling recognition begins.
_Avoid_: encoder-speed wait

**Fixed Reverse Brake**:
The fixed-duration reverse-braking action used immediately after red-block confirmation.
_Avoid_: encoder-ended braking

**Conservative Bypass Fallback**:
The default bypass action used when red-block classification cannot be completed but driving straight would risk hitting the red block.
_Avoid_: release and continue straight

**Fallback Left Bypass**:
The initial conservative fallback action that bypasses the red block on the left when classification is unavailable.
_Avoid_: dynamic fallback direction

**Vehicle Straight Pass**:
The safe straight-through action used only when model recognition confirms the vehicle class.
_Avoid_: default straight driving

**Recognition Failure**:
A red-block classification outcome where the model cannot produce a usable normal result.
_Avoid_: release to normal driving

**Invalid Recognition Retry**:
A five-frame retry window during rolling recognition when the current red block remains visible but a model inference result is unusable.
_Avoid_: immediate fallback on single invalid inference

**Voting Result**:
The class selected by the red-block model voting system when the voting system completes normally.
_Avoid_: extra confidence gate

**Current Red Block Frame**:
A red-block observation from the current perception cycle that is required for rolling recognition.
_Avoid_: stale red-block ROI

**Red Block Transition Log**:
A concise log emitted when red-block decision or motion state changes, with only low-frequency progress logs between transitions.
_Avoid_: noisy per-frame status dump

**Decision State Name**:
A red-block state name that describes the decision phase directly, such as braking, low-speed settle, model recognizing, action selected, or motion active.
_Avoid_: paused, model wait, confirmed

**Internal Red Block Boundary**:
The separation of perception, decision, and motion responsibilities inside the red-block module before splitting files.
_Avoid_: immediate file split

**Debug State Mirror**:
A read-only external mirror of internal red-block state used for telemetry and logs.
_Avoid_: externally writable state

**Element Action Suppression**:
The rule that other road-element actions do not execute while red-block decision or motion is active.
_Avoid_: resetting red-block flow from another element

**Scripted Bypass Motion**:
The existing IMU-and-encoder based bypass action sequence used by red-block motion.
_Avoid_: visual-boundary bypass rewrite

## Relationships

- **Red Block Decision** reads observations from **Red Block Perception**.
- **Red Block Decision** requests actions from **Red Block Motion**.
- **Red Block Motion** reports action progress back to **Red Block Decision**.
- **Rolling Model Recognition** starts only after **Red Block Motion** passes the **Brake Gate**.
- **Rolling Model Recognition** runs under **Low-Speed Hold**, not ordinary road-speed decision.
- **Brake Gate** is passed after **Low-Speed Settle**.
- **Low-Speed Settle** starts after **Fixed Reverse Brake** completes.
- **Conservative Bypass Fallback** is chosen when recognition cannot complete and a straight pass is unsafe.
- **Fallback Left Bypass** is the current **Conservative Bypass Fallback**.
- **Vehicle Straight Pass** is allowed only after confirmed vehicle classification.
- **Recognition Failure** always uses **Fallback Left Bypass**.
- **Invalid Recognition Retry** is allowed only while a **Current Red Block Frame** remains available.
- A completed **Voting Result** is treated as a normal recognition result without an extra vote-count threshold.
- **Rolling Model Recognition** requires a **Current Red Block Frame**; losing the red block is **Recognition Failure**.
- Red-block runtime diagnostics should use **Red Block Transition Log**.
- **Red Block Decision** should use explicit **Decision State Name** terms instead of legacy pause-oriented names.
- **Internal Red Block Boundary** should be clarified before introducing separate files.
- Internal red-block state may expose a **Debug State Mirror** for telemetry.
- Red-block activation uses **Element Action Suppression** rather than letting other elements reset the red-block flow.
- Red-block bypass should keep **Scripted Bypass Motion** during the decision-state refactor.

## Example Dialogue

> **Dev:** "The red block was detected, but the car did not slow down. Is that a Perception bug?"
> **Domain expert:** "No. If the red block observation was trusted, the slowdown belongs to Red Block Motion; Red Block Decision only requests it."

> **Dev:** "Should the car stop before classifying the object above the red block?"
> **Domain expert:** "No. It should reverse-brake, briefly settle in Low-Speed Hold, then continue with Rolling Model Recognition."

> **Dev:** "Can normal straight-road speed control run while the model votes?"
> **Domain expert:** "No. Rolling Model Recognition uses Low-Speed Hold, with only limited steering correction."

> **Dev:** "How long should the car settle after reverse braking before model recognition?"
> **Domain expert:** "Use Low-Speed Settle: five frames of Low-Speed Hold."

> **Dev:** "Can encoder readings end reverse braking early?"
> **Domain expert:** "No. Reverse braking is Fixed Reverse Brake; encoder readings are telemetry, not the transition condition."

> **Dev:** "If the car cannot slow enough for classification, should it release back to normal driving?"
> **Domain expert:** "No. Since straight driving risks hitting the red block, use Conservative Bypass Fallback."

> **Dev:** "Which fallback direction should we use before field testing dynamic selection?"
> **Domain expert:** "Use Fallback Left Bypass first, then revisit after testing."

> **Dev:** "Is straight pass safe when classification is unavailable?"
> **Domain expert:** "No. Straight pass is safe only for confirmed Vehicle Straight Pass."

> **Dev:** "What should happen if classification cannot produce a usable result?"
> **Domain expert:** "Use Fallback Left Bypass."

> **Dev:** "If one model frame is invalid but the red block is still visible, do we immediately fallback?"
> **Domain expert:** "No. Use five-frame Invalid Recognition Retry, but only while the Current Red Block Frame remains available."

> **Dev:** "Should Decision require an extra confidence rule after the voting system selects a class?"
> **Domain expert:** "No. If the voting system completes normally, use its Voting Result."

> **Dev:** "Can recognition use an old red-block ROI after the current frame loses the red block?"
> **Domain expert:** "No. Rolling Model Recognition requires a Current Red Block Frame; otherwise use Fallback Left Bypass."

> **Dev:** "Should red-block handling print every internal value every frame?"
> **Domain expert:** "No. Use Red Block Transition Log, plus sparse progress logs where needed."

> **Dev:** "Should the new decision flow keep names like paused and model wait?"
> **Domain expert:** "No. Use Decision State Name terms that match the new flow."

> **Dev:** "Should we split red-block decision and motion into separate files immediately?"
> **Domain expert:** "No. Clarify the Internal Red Block Boundary first, then split files later if needed."

> **Dev:** "Can external code keep reading red-block state for debug output?"
> **Domain expert:** "Yes, through Debug State Mirror values, but external code should not drive the state machine by writing them."

> **Dev:** "Should another element reset red-block handling while red-block motion is active?"
> **Domain expert:** "No. Use Element Action Suppression: other element actions are suppressed while red-block handling remains active."

> **Dev:** "Should this refactor replace the current bypass path with visual boundary rewriting?"
> **Domain expert:** "No. Keep Scripted Bypass Motion first; revisit visual bypass after the decision flow is stable."

## Flagged Ambiguities

- "red-block state machine" was used to mean detection, model classification, and vehicle movement together; resolved: it means **Red Block Decision** only.
- "model recognition" was ambiguous between parked and moving recognition; resolved: red-block classification uses **Rolling Model Recognition** after the **Brake Gate**.
- "low speed" was ambiguous between ordinary road speed and red-block recognition speed; resolved: recognition uses **Low-Speed Hold**.
- "Brake Gate" was ambiguous between a strict encoder-speed threshold and an image-stability settling point; resolved: it means reverse braking completed plus a short Low-Speed Hold settling period.
- "settle time" was ambiguous between encoder-based waiting and frame-based waiting; resolved: use five-frame **Low-Speed Settle**.
- "braking complete" was ambiguous between fixed duration and encoder-triggered completion; resolved: use **Fixed Reverse Brake**.
- "timeout" was ambiguous between releasing normal driving and taking a default action; resolved: red-block recognition timeout uses **Conservative Bypass Fallback**.
- "fallback bypass" was ambiguous between dynamic and fixed direction; resolved: start with **Fallback Left Bypass**.
- "straight pass" was ambiguous between normal driving and a model-confirmed action; resolved: only **Vehicle Straight Pass** may go straight.
- "failed recognition" was ambiguous between retrying, releasing, and fallback; resolved: **Recognition Failure** uses **Fallback Left Bypass**.
- "invalid model result" was ambiguous between immediate fallback and retry; resolved: use **Invalid Recognition Retry** while red block remains visible.
- "normal recognition result" was ambiguous between voting output and extra confidence gating; resolved: a completed **Voting Result** is normal.
- "red-block ROI" was ambiguous between current and cached observations; resolved: recognition uses only **Current Red Block Frame**.
- "debug output" was ambiguous between per-frame dumps and transition diagnostics; resolved: use **Red Block Transition Log**.
- Legacy state names such as paused, model wait, and confirmed no longer match the desired flow; resolved: use explicit **Decision State Name** terms.
- "refactor boundary" was ambiguous between internal organization and file splitting; resolved: start with **Internal Red Block Boundary**.
- "state flag" was ambiguous between control input and telemetry; resolved: use **Debug State Mirror** only for telemetry.
- "element exclusivity" was ambiguous between suppressing other actions and resetting red-block state; resolved: use **Element Action Suppression**.
- "bypass refactor" was ambiguous between state-machine cleanup and motion-algorithm replacement; resolved: keep **Scripted Bypass Motion** for now.
