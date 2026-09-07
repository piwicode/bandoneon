![Bandolibre keyboard](documentation/images/3dmodel_keyboard_tilt.webp)

🇬🇧 **English** | 🇫🇷 [Français](README.fr.md) | 🇪🇸 [Español](README.es.md)

# Bandolibre

An open-source MIDI controller that brings the Argentine bandoneon into the digital age.

**Bandolibre focuses on one goal:** turning key presses and bellows movement into MIDI — faithfully, instantly, and without the cost or noise of an acoustic instrument. Plug it into any computer, tablet, or phone over USB and you're ready to play, compose, or practice.

An open initiative by [L'Atelier du bandonéon libre](https://github.com/bandolibre)

---

## See it play

[![Bandolibre demo with a clarinet virtual instrument](https://img.youtube.com/vi/6s1wlRKlAk4/maxresdefault.jpg)](https://youtu.be/6s1wlRKlAk4)

Watch the demos: [with a clarinet virtual instrument](https://youtu.be/6s1wlRKlAk4) · [with strings virtual instruments](https://youtu.be/nJ0j7DtbYDk).

---

## Who is it for?

- **Students** who don't own an instrument yet and want to start learning
- **Players** who want to practice silently — at home, on the road, at any hour
- **Composers** entering scores into notation software one note at a time
- **Musicians** triggering synths and samplers on stage or in the studio

---

## How it works

- **142-key Rheinische Lage layout** — both hands, exact fingering of the Argentine bandoneon
- **Hall-effect sensors** on every key — no mechanical contact, no wear, scanned 2400 times per second
- **Blade spring + sensor pair** for the bellows — measures push/pull effort and emits MIDI CC#11 (Expression), just like the real thing
- **Two 6.35 mm expression pedal inputs** — compatible with M-Audio EX-P pedals; pedal 1 sends CC#1 (Modulation), pedal 2 sends CC#4 (Foot Controller)
- **USB-MIDI** out of the box — plug into any DAW, notation app, or synth; no drivers needed

---

## Features

**Table mode** — a button toggles table mode: keys fire immediately at fixed velocity, no bellows movement required. Handy for entering a score note by note without working the bellows.

**Play anywhere** — Bandolibre is bus-powered over USB; any phone, tablet, or laptop with a soft synth becomes the sound engine. A small USB hub with a headphone jack and a power pass-through gives you audio, charging, and MIDI from a single cable — tested and pocket-sized.

**Adaptable** — standard USB-MIDI is compatible with the whole adapter ecosystem: plug in a USB Bluetooth MIDI adapter to play wirelessly, or a USB-to-DIN-5 adapter to drive vintage hardware synths.

---

## Anyone can build it

This is a fully open DIY project. PCBs are designed for economic JLCPCB two-layer fabrication — affordable and easy to order. The mechanical parts are 3D-printable (a FabLab near you works great). Firmware is open-source and flashable with a standard [ST-LINK probe](https://www.st.com/en/development-tools/stlink-v3minie.html) and a [TC-2070-IDC-050](https://www.tag-connect.com/product/tc2070-idc-050) cable.

Building one costs about as much as a decent MIDI keyboard or a good pair of studio headphones like the DT-770 Pro.

---

## Bill of Materials

| Part | Qty | Notes |
|------|-----|-------|
| 3D-printed body parts + 71 keys | — | ~880 g filament, ~24 h print time, [STEP files](https://github.com/bandolibre/bandolibre.github.io/releases)|
| Electronics board: Main, left and right PCBs  | 1 | [EasyEDA design](boards), [Gerber files](https://github.com/bandolibre/bandolibre.github.io/releases) |
| Hall-effect switches | 71 | [GATERON Low Profile Magnetic Jade HE](https://www.gateron.com/products/gateron-low-profile-magnetic-jade-switch?VariantsId=10872) |
| Bellows spring strip | 1 | [65Mn spring steel, 1.2 × 40 × 300 mm](https://fr.aliexpress.com/item/1005006952720032.html?spm=a2g0o.order_list.order_list_main.17.3cfd1802U67eT2&gatewayAdapt=glo2fra) |
| Permanent magnets 14.5 × 6 × 2 mm | 2 |  |
| M4 × 30 socket head cap, stainless A2 | 4 | [supplier](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36756-927484-vis-metaux-inox-a2-chc-btr-cle-de-3-hc3-m4x40-filetee-sur-22.html#/267-conditionnement-200_pieces)|
| M4 × 10 × 0.8 flat washer, stainless A2 | 4 | [supplier](https://www.vis-express.fr/rondelle-plate-m-inox-a2-nfe-25513vs-nfe25513-grade-c/36925-940576-rondelle-plate-m4x10x08-m-inox-a2.html#/267-conditionnement-200_pieces)|
| M3 × 6 socket head cap, stainless A2 | 19 | [supplier](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36726-2595617-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x6-filetage-total.html#/21-conditionnement-1_piece)|
| M3 × 8 socket head cap, stainless A2 | 11 | [supplier](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36728-2610009-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x8-filetage-total.html#/21-conditionnement-1_piece) |
| M3 × 12 socket head cap, stainless A2 | 4 | [supplier](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36731-2616466-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x12-filetage-total.html#/21-conditionnement-1_piece) |
| M3 × 18 socket head cap, stainless A2 | 4 | [supplier](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36734-2596394-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x18-filetage-total.html#/21-conditionnement-1_piece) |
| Adhesives | 2 drops | Loctite 480 |
| USB Type-B cable | 1 | |
| Ribbon cable, FC 1.27 mm, 12P (2×6), 10 cm | 2 | [supplier](https://fr.aliexpress.com/item/1005005058041580.html) |

The full list, including tools and consumables, is in [`documentation/assembly_instructions.md`](documentation/assembly_instructions.md).

They are best built in batches of five — JLCPCB minimum orders make that the natural unit. Team up with friends or reach out to [L'Atelier du bandonéon libre](https://github.com/bandolibre) to express interest in a community build.

At fifty units, PCB fabrication and Gateron switches — the two biggest line items — drop by half again.

---

## Design

![overview](documentation/images/3dmodel_overview.webp)

The mechanical parts are modeled in Onshape — [the full 3D model](https://cad.onshape.com/documents/313e70e978bf056a8dd7d76c/v/5c5fbc4088ac379c1bd1b53a/e/c6a89cb028bdc195ff70596f?showReturnToWorkspaceLink=tru) is public and interactive. Reference [photography](keyboard_picture/) of real instruments was used to accurately reproduce the shape, key placement, and keyboard tilt of both hands.
![handle_layout](documentation/images/3dmodel_handle.webp)
![keyboard_layout](documentation/images/3dmodel_keyboard_layout.webp)

The PCBs are designed with EasyEDA.
![3d_pcb](documentation/images/pcb_main_board_3d.png)

The bellows is replaced by a **blade spring instrumented with two Hall-effect sensors** that read its flexion. Load cell solutions were ruled out early — they are too rigid and remove the tactile feedback players rely on to feel and modulate their effort — it plays like pressing on a wall. The blade spring preserves that proprioceptive feedback while being simple and durable. Blade thickness can be chosen to tune the instrument's stiffness, from light to firm.

![blade spring](documentation/images/bandolibre_blade.webp)

A sensitivity selector button cycles through three amplification levels so the player can adjust how much bellows travel is needed to reach full expression — useful for quiet practice or a stiffer spring.

For a detailed breakdown of the firmware behavior and controls, see [`documentation/features.md`](documentation/features.md).

---

## Build it

- **Hardware & assembly:** see [`documentation/assembly_instructions.md`](documentation/assembly_instructions.md)
- **Firmware:** see [`code/`](code/) — build with `just build` and `just flash`, flash via ST-LINK

---

## Sound

The best match so far is the **SWAM** family of simulation instruments from [Audio Modeling](https://audiomodeling.com/): they are physically modelled rather than sampled, so CC#11 continuously drives the model itself. Bellows intensity comes out as real dynamics — timbre changing with pressure, notes swelling and dying under the bellows — instead of a volume fade over a fixed sample. [Native Instruments Session Strings](https://www.native-instruments.com/en/products/komplete/cinematic/session-strings-2) also works very well.

Modern bandoneon libraries, oddly enough, work poorly for us. They either assume the wrong keyboard layout — chromatic or accordion mappings, with no push/pull distinction — or offer only shallow CC#11 support, with dynamics baked into velocity-triggered samples that the bellows cannot reshape once a note has started.

For practice — and especially on a phone — a simple bandoneon soundfont is enough: [European Bandoneon V2.5 by Jörg Bleymehl](https://musical-artifacts.com/artifacts/1862) gives good results. It loads in any SF2-capable player, costs almost nothing in CPU, and turns a phone or tablet into a usable practice rig without a laptop or a plugin host.

So we are actively looking for virtual instruments with deep expressive support — rich polyphony and CC#11 as a primary articulation driver. If you know of one, or want to help build something tailored to the bandoneon, suggestions and contributions are very welcome.

---

## Where we are

<p>
  <img src="documentation/images/bandolibre_overview.webp" alt="Bandolibre overview" width="49%">
  <img src="documentation/images/bandolibre_main_module.webp" alt="Bandolibre main module" width="49%">
</p>

Five units are built and working. The firmware handles all 142 keys, bellows push/pull, pedals, and MIDI output reliably. These instruments are currently on loan to bandoneon teachers who are giving us hands-on feedback while we polish the software.

The 3D models and PCB designs are solid — no rework planned there. The active focus right now is tuning the bellows simulation: getting the inertia model right so that short notes feel like the real instrument, not like a sensor.

There is a lot to explore on the software side. Because Hall-effect sensors measure key position continuously — not just on/off — the firmware has access to the full travel of every key at all times. This opens the door to **MPE (MIDI Polyphonic Expression)**: per-note pressure, slide, and lift curves, independently for each of the 142 keys simultaneously.

Bandolibre stays a DIY project: the design is open and anyone can build one. What is missing is the paperwork that would let the association handle a transaction at all — passing on boards, a kit, or a finished instrument to someone who asks. That is a few months away, so there is nothing to arrange today. If you'd like to hear from us when it becomes possible, [leave your details on this form](https://forms.gle/amgxEX4XTy9Jfd538). It also helps us see how many people are interested, and what they'd play it for. Your answers stay with the association and are only used to contact you about Bandolibre.

---

## Community

Questions, ideas, or just curious?
Join [L'Atelier du bandonéon libre](https://bandolibre.github.io).

The main board communicates digitally with the wing boards and can support any layout. It is possible to design a new keyboard for a different system — Rheinische Lage, Club, Einheitsbandoneon, Peguri, Manouri — and reuse the main board.

Working on something similar? Let the association know — we'd love to connect.

---

## Further readings

- [Other electronic bandoneon projects](documentation/other-projects.md)

---

## License

[![CC BY-NC-SA 4.0](https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-nc-sa.eu.svg)](LICENSE.md)

Free to build, modify, and share for non-commercial use.
