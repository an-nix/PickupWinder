# Déploiement manuel PRU (PRU1 motor-control)

Ce fichier décrit la procédure manuelle pour déployer le firmware PRU0 `am335x-pru0-fw`
qui pilote les deux moteurs (A + B) sur PRU1 selon le mapping P8 canonique.

Important
- Le firmware PRU0 **possède** le compteur IEP (il appelle `IEP_INIT()`).
- PRU1 exécute maintenant le firmware moteur; PRU0 gère l'orchestration et la communication.
- Les signaux ENABLE sont actifs LOW (0 = driver ON).

Rappel GPIO (PRU1 R30 — P8 GPMC group, confirmed pinctrl debugfs)
- P8_45  → STEP_A (R30[0])          Motor A
- P8_46  → STEP_B (R30[1])          Motor B (spindle câblé ici)
- P8_43  → DIR_A  (R30[2])          Motor A
- P8_44  → DIR_B  (R30[3])          Motor B (spindle câblé ici)
- P8_41  → EN_A   (R30[4]) active-low   Motor A
- P8_42  → EN_B   (R30[5]) active-low   Motor B (spindle câblé ici)

Endstops (PRU0 R31 — P9 header)
- P9_28  → ENDSTOP_1 (R31[3])  (PRU0 orchestrateur, pull-up)
- P9_30  → ENDSTOP_2 (R31[2])  (PRU0 orchestrateur, pull-up)

## Modèle d'accélération (Klipper-style interval += add)

Le firmware PRU utilise le même modèle d'accélération que Klipper (stepper.c) :

```
À chaque pas (front montant STEP) :
  interval += add
  count--
Quand count == 0 → vitesse constante (coast)
```

Le host (daemon ou test firmware) pré-calcule trois valeurs par segment :
- **interval** — demi-période IEP initiale du segment
- **add** — delta signé ajouté à interval à chaque pas
- **count** — nombre de pas dans le segment

Profil trapézoïdal = 3 segments : accel (add < 0) + cruise (add = 0) + decel (add > 0).

Formule pour `add` :
```
add = (end_iv - start_iv) / ramp_steps
```
Une seule division, faite côté host. Le PRU ne fait qu'une addition entière par pas.

Référence : Klipper `resources/klipper/stepper.c` (stepper_event_edge)

## Déploiement automatisé recommandé

Depuis la machine de développement :

```bash
cd /home/nicolas/Documents/PlatformIO/Projects/PickupWinder/src/pru
./scripts/deploy.sh --host BBB_IP --spin-test
```

Ce script :
- compile les firmwares + l'overlay,
- copie le payload sur la BBB,
- installe `/lib/firmware/am335x-pru0-fw`, `/lib/firmware/am335x-pru1-fw` et le `.dtbo`,
- installe `/usr/local/bin/load_pru.sh`, `/usr/local/bin/test_pru0_motor.sh` et `/usr/local/bin/pru_spin_test.py`,
- installe/active le service `pickupwinder-pru.service`,
- démarre PRU0 + PRU1,
- peut lancer un test de rotation du moteur broche.

## Déploiement manuel

1) Construire localement (sur le PC)

```bash
cd /home/nicolas/Documents/PlatformIO/Projects/PickupWinder/src/pru
make         # compile les firmwares (motor + orchestration)
make dtbo    # compile les .dtbo depuis dts/ (optionnel pour pinmux persistant)
```

2) Copier les fichiers sur la BBB (ex. vers /tmp)

```bash
scp build/am335x-pru0-fw debian@BBB_IP:/tmp/
scp build/dtbo/pickup-winder-p8-steppers.dtbo debian@BBB_IP:/tmp/   # si utilisé
scp test_pru0_motor.sh debian@BBB_IP:/tmp/
scp scripts/load_pru.sh debian@BBB_IP:/tmp/
scp scripts/pru_spin_test.py debian@BBB_IP:/tmp/
scp scripts/pickupwinder-pru.service debian@BBB_IP:/tmp/
```

3) Installer sur la BBB (actions root)

```bash
# copier firmware + overlay + scripts
sudo cp /tmp/am335x-pru0-fw /lib/firmware/
sudo cp /tmp/am335x-pru1-fw /lib/firmware/
sudo cp /tmp/pickup-winder-p8-steppers.dtbo /lib/firmware/   # optionnel
sudo cp /tmp/test_pru0_motor.sh /usr/local/bin/
sudo cp /tmp/load_pru.sh /usr/local/bin/
sudo cp /tmp/pru_spin_test.py /usr/local/bin/
sudo chmod +x /usr/local/bin/test_pru0_motor.sh
sudo chmod +x /usr/local/bin/load_pru.sh
sudo chmod +x /usr/local/bin/pru_spin_test.py

# optionnel : service de démarrage
sudo cp /tmp/pickupwinder-pru.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pickupwinder-pru.service
```

4a) Méthode simple (recommandée pour tests rapides) — utiliser le script

```bash
sudo /usr/local/bin/test_pru0_motor.sh
```

Le script :
- met les pins moteur sur le mapping PRU1 (`pruout`) et les endstops sur PRU0 (`pruin`) via `config-pin`
- arrête PRU0/PRU1, assigne `am335x-pru0-fw` + `am335x-pru1-fw` et démarre les deux PRUs
- affiche l'état final

Test de rotation spindle :

```bash
sudo /usr/local/bin/pru_spin_test.py --rpm 20 --seconds 6 --forward 1
```

4b) Méthode manuelle (remoteproc + config-pin)

```bash
# config des pins (en local sur la BBB)
for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46; do
  sudo config-pin "$pin" pruout
done
for pin in P9_28 P9_30; do
  sudo config-pin "$pin" pruin
done

# remoteproc (vérifier le numéro remoteproc sur votre kernel)
# remoteproc1 est habituel pour PRU0; adaptez si nécessaire
echo stop  | sudo tee /sys/class/remoteproc/remoteproc1/state
echo am335x-pru0-fw | sudo tee /sys/class/remoteproc/remoteproc1/firmware
echo start | sudo tee /sys/class/remoteproc/remoteproc1/state

# vérifier
cat /sys/class/remoteproc/remoteproc1/state
sudo dmesg | tail -20
```

5) Restaurer les pins (après test)

```bash
# via le script
sudo /usr/local/bin/test_pru0_motor.sh --restore

# ou manuellement
for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46 P8_15 P8_16; do
  sudo config-pin "$pin" default
done
```

6) Vérifications utiles
- `cat /sys/class/remoteproc/remoteproc1/state` → doit afficher `running`.
- `dmesg | tail -20` → messages kernel relatifs au chargement du firmware.
- Si vous avez des problèmes de démarrage, arrêtez PRU (`echo stop`) et relisez
  les logs : `dmesg`.

### Si `config-pin` / paquet `bb-cape-overlays` n'est pas disponible

Sur certaines images Debian/BBB le paquet `bb-cape-overlays` peut ne pas être
présent dans les dépôts; dans ce cas `config-pin` renvoie une erreur
"failed to set pruout". Trois solutions possibles :

1) Installer `config-pin` depuis le dépôt `bb.org-overlays`

```bash
sudo apt update
sudo apt install git
git clone https://github.com/beagleboard/bb.org-overlays.git /tmp/bb.org-overlays
sudo cp /tmp/bb.org-overlays/tools/config-pin /usr/local/bin/
sudo chmod +x /usr/local/bin/config-pin
# vérifier
which config-pin && config-pin -q P8_45 || echo "config-pin non opérationnel"
```

2) Charger le `.dtbo` au boot (recommandé pour production)

Copiez le fichier `.dtbo` dans `/lib/firmware/` et activez l'overlay au démarrage
via `/boot/uEnv.txt` (méthode u-boot) ou via votre mécanisme d'overlays :

```text
enable_uboot_overlays=1
uboot_overlay_addr4=/lib/firmware/pickup-winder-p8-steppers.dtbo
```

Puis reboot. Cette méthode applique le pinmux très tôt pendant l'init kernel
et réduit fortement la fenêtre de risque. Pour tester sans reboot, si votre
kernel expose encore `capemgr` :

```bash
echo pickup-winder-p8-steppers | sudo tee /sys/devices/platform/bone_capemgr/slots
dmesg | tail -n 40
```

3) Mitigation matérielle (sécurité maximale)

Ajouter des résistances pull-up externes (par ex. 10 kΩ) sur les lignes
`EN_A` et `EN_B` vers le +V afin de forcer `EN = HIGH` (drivers
désactivés) tant que le pinmux PRU n'est pas appliqué. C'est la protection la
plus sûre quel que soit le mécanisme de boot.

Diagnostics rapides :

```bash
which config-pin || echo "config-pin absent"
ls /sys/devices/platform | grep -i cape || true
dmesg | tail -n 40
```


Notes additionnelles
- Pour un usage persistant, installez le `.dtbo` et ajoutez la surcharge
  via la méthode d'activation DT appropriée à votre distribution (ex: `uEnv.txt`
  ou `config-pin` au démarrage). La méthode exacte dépend de la version du
  kernel et des scripts de démarrage de la BBB.
- Ne lancez pas deux firmwares faisant chacun la consommation de la même
  `cmd_rhead`/anneau de commandes simultanément — cela corromprait le protocole.

Si vous voulez, je peux :
- ajouter un `make install`/`make deploy` plus sélectif,
- automatiser le commit + push de `PRU_DEPLOY.md` (je peux le faire maintenant).
