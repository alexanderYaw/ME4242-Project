# Six direction candidates

Same spec (`spec.md`), same simulated data. Three are built in full (marked ★); the other three are
alternates shown as thumbnails in `thumbs.html`.

Seconds roulette: `date +%S` = 11 → web style 11 % 20 + 1 = #12 Warm Editorial; east style 11 % 12 + 1 =
#12 Japanese Lifestyle Magazine Dense.

---

## ★ A · Ward monitor — real reference: Philips IntelliVue patient monitors

- **Logic & source:** real reference, same medium (safety-critical live-monitoring equipment UI). Verified:
  Philips' IntelliVue alarm documentation (high = white text on red, medium = yellow, low = cyan; an
  acknowledged alarm keeps reminding until the condition clears). Screen layout **not viewed in person**
  this session; borrowing the documented alarm logic plus the well-known monitor composition.
- **What is borrowed:** a full-width alarm band directly under the global bar, alarms as text on a colour
  field, ordered by priority, acknowledged-but-active kept visible; a right-hand column of
  "parameter rows" (label left, large tabular number right, limits/units small) like monitor numerics;
  the dominant waveform area becomes the dominant workspace view. User's own recorded preference for
  monitoring screens: light paper-and-ink palette, colour reserved for status.
- **Hero material:** the live numbers themselves (tip position, clearance, motor angle/current) as big
  tabular numerals, plus the one dominant view.
- **Skeleton:** global bar → alarm band → [dominant view with view switcher + secondary thumbnail | parameter
  column: tip, clearance, motors 0–3 rows] → task switch (Operate / Motors / Setup) in the bar.
- **Where the nerve goes:** the alarm band is the only saturated colour on a near-white screen.
- **Form comes from:** the arm is "monitored" like a patient: several vital readings, one place for alarms.
- Day: ink on warm-grey paper. Night: dim charcoal with the same hierarchy (not inverted neon).

## ★ B · Pilot — real reference: DJI Pilot 2 (enterprise drone flight app used by public-safety teams)

- **Logic & source:** real reference, same medium (field-operator control app on a tablet). Verified:
  DJI's Pilot 2 release material — one-tap access to aircraft/payload controls, switching camera views
  from the controller, navigation display with home point and obstacles, alert notifications. Exact screen
  layout **not viewed in person**; picture-in-picture behaviour unconfirmed, so the swap is my design choice.
- **What is borrowed:** the view is the whole screen; telemetry sits on the view edges, never in panels;
  big thumb-reachable controls on the left and right edges of a landscape tablet; a secondary view as a
  small inset you tap to swap; alerts drop in at the top centre.
- **Hero material:** the dominant view (camera / top / side / 3D) edge to edge.
- **Skeleton:** full-bleed view; left edge = joystick + mapping; right edge = target, Move tip, tip Stop;
  top strip = global bar; Motors and Setup open as a bottom sheet over the view (view stays visible).
- **Where the nerve goes:** nothing is boxed; the controls float on the view with solid, opaque plates.
- **Form comes from:** the operator is effectively flying a remote effector by camera.
- Night is the default; Day raises the plates to light and the view keeps its own contrast.

## ★ C · Floor plan — best designer: Otl Aicher (Munich 1972 / Lufthansa systems)

- **Logic & source:** best designer. Aicher's thinking: a strict grid, a small set of pictograms that
  stand for whole subsystems, colour as a code with fixed meanings, a narrow condensed type set in a few
  sizes, information placed where it physically belongs.
- **What is borrowed:** systematic coding (each motor has a fixed numeral block; status colour only means
  status); grid with fixed column widths; pictograms + words for every task area; flat colour fields, no
  shadows; the motor overview drawn **in the robot's physical layout from above** (1 top-left, 0 top-right,
  2 bottom-left, 3 bottom-right) with pair lines A (0–2) and B (1–3).
- **Hero material:** the 2×2 physical motor map that doubles as status board and joystick reference.
- **Skeleton:** global bar; left task rail with pictograms (Operate / Motors / Setup); Operate = dominant
  view (left, large) + a fixed control column on the right (move, joystick, stops); the 2×2 motor map is
  always on screen under the control column.
- **Where the nerve goes:** the motor map — status blocks big enough to read across a room.
- **Form comes from:** four motors around a cable-driven arm, seen from above.
- Day: white with black rules; Night: black with grey rules; the status colours keep their meaning.

---

## Alternates (thumbnails only)

## D · Faceplate — best designer: Dieter Rams (Braun)
- Borrowed: instrument faceplate calm, one functional accent colour, scale markings, controls aligned
  in rows of equal size, labels under controls. Hero: a single horizontal "instrument" strip of motor
  scales. Skeleton: view top, control strip bottom. Nerve: everything else silent, one orange for "act".
  Form: the arm's motor drums as rotary scales. Not chosen: closest to C in temperament and weaker at
  alerting (Braun calm fights alarm urgency).

## E · Warm Editorial — roulette web #12
- Borrowed: cream paper, serif headings, terracotta accent, editorial columns. Hero: a large serif
  "status sentence" ("Arm is holding. Tip 12 mm from rod."). Skeleton: two text columns + view.
  Not chosen: serif display slows glance reading under stress, terracotta collides with warning amber, and
  it is the generic AI-default family.

## F · Annotated plate — roulette east #12 (Japanese lifestyle magazine density)
- Borrowed: dense annotated diagram, numbered call-outs on the view, data grid with thick rules, marker
  underline for the one thing that matters. Hero: the top view covered in hand-placed annotations.
  Not chosen: density is the opposite of the brief (reduce clutter); the call-out idea is kept as a
  possible detail for whichever direction wins.
