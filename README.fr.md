![Bandolibre keyboard](documentation/images/3dmodel_keyboard_tilt.webp)

🇬🇧 [English](README.md) | 🇫🇷 **Français** | 🇪🇸 [Español](README.es.md)

# Bandolibre

Un contrôleur MIDI open-source qui fait entrer le bandonéon argentin dans l'ère numérique.

**Bandolibre n'a qu'un seul objectif :** convertir en MIDI les pressions sur les touches et les mouvements du soufflet — fidèlement, instantanément, sans les contraintes de coût ou de bruit d'un instrument acoustique. Branchez-le sur un ordinateur, une tablette ou un téléphone via USB et vous êtes prêt à jouer, composer ou vous entraîner.

Une initiative ouverte de [L'Atelier du bandonéon libre](https://github.com/bandolibre)

---

## À l'écoute

[![Démo du Bandolibre avec une clarinette virtuelle](https://img.youtube.com/vi/6s1wlRKlAk4/maxresdefault.jpg)](https://youtu.be/6s1wlRKlAk4)

Voir les démos : [avec une clarinette virtuelle](https://youtu.be/6s1wlRKlAk4) · [avec des instruments virtuels à cordes](https://youtu.be/nJ0j7DtbYDk).

---

## Pour qui ?

- **Les élèves** qui n'ont pas encore d'instrument et veulent commencer à apprendre
- **Les musiciens** qui veulent s'entraîner en silence — à la maison, en voyage, à toute heure
- **Les compositeurs** qui saisissent des partitions note par note dans un logiciel de notation
- **Les musiciens de scène** qui pilotent synthétiseurs et sampleurs en concert ou en studio

---

## Comment ça fonctionne

- **Clavier Rheinische Lage à 142 tons** — les deux mains, doigtés exacts du bandonéon argentin
- **Capteurs à effet Hall sur chaque touche** — pas de contact mécanique, aucune usure, 2400 lectures par seconde
- **Lame-ressort + capteur** pour le soufflet — mesure l'effort de poussée et de traction et émet MIDI CC#11 (Expression), comme le vrai instrument
- **Deux entrées pédale d'expression 6,35 mm** — compatibles M-Audio EX-P ; pédale 1 envoie CC#1 (Modulation), pédale 2 envoie CC#4 (Contrôleur de pied)
- **USB-MIDI** nativement — branchez sur n'importe quel DAW, logiciel de notation ou synthé ; aucun pilote nécessaire

---

## Fonctionnalités

**Mode table** — un bouton active le mode table : les touches se déclenchent immédiatement à vélocité fixe, sans mouvement de soufflet. Idéal pour saisir une partition note par note sans actionner le soufflet.

**Jouez partout** — Bandolibre est alimenté par le bus USB ; n'importe quel téléphone, tablette ou ordinateur portable avec un synthé logiciel devient le moteur sonore. Un petit hub USB avec prise casque et pass-through d'alimentation vous donne audio, charge et MIDI en un seul câble — testé et tient dans une poche.

**Adaptable** — le standard USB-MIDI est compatible avec tout l'écosystème d'adaptateurs : branchez un adaptateur USB Bluetooth MIDI pour jouer sans fil, ou un adaptateur USB-vers-DIN-5 pour piloter des synthétiseurs matériels vintage.

---

## Tout le monde peut le construire

C'est un projet DIY entièrement ouvert. Les PCB sont conçus pour la fabrication économique JLCPCB deux couches — abordables et faciles à commander. Les pièces mécaniques sont imprimables en 3D (un FabLab près de chez vous convient parfaitement). Le firmware est open-source et peut être flashé avec une sonde [ST-LINK](https://www.st.com/en/development-tools/stlink-v3minie.html) standard et un câble [TC-2070-IDC-050](https://www.tag-connect.com/product/tc2070-idc-050).

Construire un Bandolibre coûte à peu près autant qu'un bon clavier MIDI ou une bonne paire de casques de studio comme le DT-770 Pro.

---

## Nomenclature

| Pièce | Qté | Notes |
|-------|-----|-------|
| Pièces imprimées 3D + 71 touches | — | ~880 g de filament, ~24 h d'impression, [fichiers STEP](https://github.com/bandolibre/bandolibre.github.io/releases)|
| Cartes électroniques : principale, gauche et droite | 1 | [Conception EasyEDA](boards), [fichiers Gerber](https://github.com/bandolibre/bandolibre.github.io/releases) |
| Interrupteurs à effet Hall | 71 | [GATERON Low Profile Magnetic Jade HE](https://www.gateron.com/products/gateron-low-profile-magnetic-jade-switch?VariantsId=10872) |
| Lame-ressort soufflet | 1 | [Acier à ressort 65Mn, 1,2 × 40 × 300 mm](https://fr.aliexpress.com/item/1005006952720032.html?spm=a2g0o.order_list.order_list_main.17.3cfd1802U67eT2&gatewayAdapt=glo2fra) |
| Aimants permanents 14,5 × 6 × 2 mm | 2 | |
| Vis CHC M4 × 30, inox A2 | 4 | [fournisseur](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36756-927484-vis-metaux-inox-a2-chc-btr-cle-de-3-hc3-m4x40-filetee-sur-22.html#/267-conditionnement-200_pieces)|
| Rondelle plate M4 × 10 × 0,8, inox A2 | 4 | [fournisseur](https://www.vis-express.fr/rondelle-plate-m-inox-a2-nfe-25513vs-nfe25513-grade-c/36925-940576-rondelle-plate-m4x10x08-m-inox-a2.html#/267-conditionnement-200_pieces)|
| Vis CHC M3 × 6, inox A2 | 19 | [fournisseur](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36726-2595617-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x6-filetage-total.html#/21-conditionnement-1_piece)|
| Vis CHC M3 × 8, inox A2 | 11 | [fournisseur](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36728-2610009-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x8-filetage-total.html#/21-conditionnement-1_piece) |
| Vis CHC M3 × 12, inox A2 | 4 | [fournisseur](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36731-2616466-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x12-filetage-total.html#/21-conditionnement-1_piece) |
| Vis CHC M3 × 18, inox A2 | 4 | [fournisseur](https://www.vis-express.fr/vis-metaux-inox-a2-chc-btr-cle-de-8-hc8-filetage-total-din-912-din-912-iso-4762/36734-2596394-vis-metaux-inox-a2-chc-btr-cle-de-25-hc25-m3x18-filetage-total.html#/21-conditionnement-1_piece) |
| Adhésifs | 2 gouttes | Loctite 480 |
| Câble USB Type-B | 1 | |
| Nappe, FC 1.27 mm, 12P (2×6), 10 cm | 2 | [fournisseur](https://fr.aliexpress.com/item/1005005058041580.html) |

La liste complète, outillage et consommables compris, se trouve dans [`documentation/assembly_instructions.md`](documentation/assembly_instructions.md).

La construction par lot de cinq est idéale — les commandes minimum JLCPCB en font l'unité naturelle. Faites équipe avec des amis ou contactez [L'Atelier du bandonéon libre](https://github.com/bandolibre) pour manifester votre intérêt pour une construction collective.

À cinquante unités, la fabrication des PCB et les interrupteurs Gateron — les deux postes les plus importants — baissent encore de moitié.

---

## Conception

![overview](documentation/images/3dmodel_overview.webp)

Les pièces mécaniques sont modélisées dans Onshape — [le modèle 3D complet](https://cad.onshape.com/documents/313e70e978bf056a8dd7d76c/v/5c5fbc4088ac379c1bd1b53a/e/c6a89cb028bdc195ff70596f?showReturnToWorkspaceLink=tru) est public et interactif. Des [photographies de référence](keyboard_picture/) d'instruments réels ont été utilisées pour reproduire fidèlement la forme, le placement des touches et l'inclinaison du clavier des deux mains.
![handle_layout](documentation/images/3dmodel_handle.webp)
![keyboard_layout](documentation/images/3dmodel_keyboard_layout.webp)

Les PCB sont conçus avec EasyEDA.
![3d_pcb](documentation/images/pcb_main_board_3d.png)

Le soufflet est remplacé par une **lame-ressort instrumentée de deux capteurs à effet Hall** qui lisent sa flexion. Les solutions à base de cellule de charge n'ont pas été retenues car elles sont trop rigides et n'aident pas à ressentir et moduler la pression — ça joue comme si on appuyait sur un mur. La lame-ressort préserve ce retour proprioceptif tout en étant simple et durable. L'épaisseur de la lame peut être choisie pour régler la rigidité de l'instrument, du plus souple au plus ferme.

![3d_pcb](documentation/images/bandolibre_blade.webp)

Un bouton sélecteur de sensibilité permet de cycler entre trois niveaux d'amplification pour ajuster la course de soufflet nécessaire à l'expression maximale — utile pour jouer doucement ou avec une lame plus rigide.

Pour un détail du comportement du firmware et des commandes, voir [`documentation/features.md`](documentation/features.md).

---

## Construction

- **Matériel et assemblage :** voir [`documentation/assembly_instructions.md`](documentation/assembly_instructions.md)
- **Firmware :** voir [`code/`](code/) — construire avec `just build` et `just flash`, flasher via ST-LINK

---

## Le son

Ce qui fonctionne le mieux jusqu'ici, ce sont les instruments de simulation de la gamme **SWAM** d'[Audio Modeling](https://audiomodeling.com/) : ils reposent sur une modélisation physique plutôt que sur des échantillons, et le CC#11 pilote donc le modèle lui-même en continu. L'intensité du soufflet ressort en véritables nuances — un timbre qui change avec la pression, des notes qui enflent et s'éteignent sous le soufflet — au lieu d'un simple fondu de volume sur un échantillon figé. [Native Instruments Session Strings](https://www.native-instruments.com/en/products/komplete/cinematic/session-strings-2) donne également d'excellents résultats.

Curieusement, les bibliothèques de bandonéon modernes nous conviennent mal. Soit elles supposent une disposition de clavier inadaptée — mappages chromatiques ou d'accordéon, sans distinction poussé/tiré — soit elles n'offrent qu'un support superficiel du CC#11, avec des nuances figées dans des échantillons déclenchés à la vélocité, que le soufflet ne peut plus remodeler une fois la note lancée.

Pour travailler — et en particulier sur téléphone — une simple soundfont de bandonéon suffit : [European Bandoneon V2.5 de Jörg Bleymehl](https://musical-artifacts.com/artifacts/1862) donne de bons résultats. Elle se charge dans n'importe quel lecteur SF2, ne coûte presque rien en CPU, et transforme un téléphone ou une tablette en instrument de travail utilisable, sans ordinateur portable ni hôte de plugins.

Nous cherchons donc activement des instruments virtuels avec un support expressif profond — polyphonie riche et CC#11 comme pilote d'articulation principal. Si vous en connaissez un, ou souhaitez aider à construire quelque chose de taillé pour le bandonéon, suggestions et contributions sont les bienvenues.

---

## Où en sommes-nous

<p>
  <img src="documentation/images/bandolibre_overview.webp" alt="Bandolibre overview" width="49%">
  <img src="documentation/images/bandolibre_main_module.webp" alt="Bandolibre main module" width="49%">
</p>

Cinq unités sont construites et fonctionnelles. Le firmware gère les 142 touches, le soufflet poussé/tiré, les pédales et la sortie MIDI de manière fiable. Ces instruments sont actuellement prêtés à des professeurs de bandonéon qui nous donnent leurs retours terrain pendant que nous peaufinons le logiciel.

Les modèles 3D et les conceptions de PCB sont solides — aucune révision prévue. L'attention se porte en ce moment sur le réglage de la simulation du soufflet : faire en sorte que le modèle d'inertie rende les notes courtes aussi vivantes que sur le vrai instrument.

Il reste beaucoup à explorer côté logiciel. Parce que les capteurs à effet Hall mesurent en continu la position des touches — pas seulement ouvert/fermé — le firmware a accès à la course complète de chaque touche à tout moment. Cela ouvre la voie au **MPE (MIDI Polyphonic Expression)** : courbes de pression, de glissé et de relâché par note, indépendamment pour chacune des 142 touches simultanément.

Bandolibre reste un projet DIY : les plans sont ouverts et chacun peut en construire un. Ce qui manque, c'est le cadre administratif qui permettrait à l'association de conclure une transaction — céder des cartes, un kit ou un instrument fini à quelqu'un qui le demande. C'est encore à quelques mois, il n'y a donc rien à organiser aujourd'hui. Si vous souhaitez que nous vous prévenions quand ce sera possible, [laissez vos coordonnées sur ce formulaire](https://forms.gle/amgxEX4XTy9Jfd538). Cela nous aide aussi à voir combien de personnes sont intéressées, et ce qu'elles aimeraient en jouer. Vos réponses restent au sein de l'association et servent uniquement à vous recontacter au sujet de Bandolibre.

---

## Communauté

Questions, idées, ou simplement curieux ?
Rejoignez [L'Atelier du bandonéon libre](https://bandolibre.github.io).

La carte principale communique numériquement avec les cartes de ailes et peut prendre en charge de noeaux types de clavier. Il est possible de concevoir un nouveau clavier pour un système différent — Einheitsbandoneon, Peguri, Manouri — et de réutiliser la carte principale.

Vous travaillez sur quelque chose de similaire ? Faites-le savoir à l'association — nous serions ravis d'échanger.

---

## Pour aller plus loin

- [Autres projets de bandonéon électronique](documentation/other-projects.md)

---

## Licence

[![CC BY-NC-SA 4.0](https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-nc-sa.eu.svg)](LICENSE.md)

Libre de construire, modifier et partager pour un usage non commercial.
