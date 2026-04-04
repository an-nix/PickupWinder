# Déploiement manuel PRU (PRU1 dual-stepper)

Ce fichier décrit la procédure manuelle pour déployer le firmware PRU1 `am335x-pru1-fw`
qui pilote les deux moteurs (spindle + lateral) sur les broches P8_41..P8_46.

Important
- Le firmware `pru1_dual_stepper` fait que PRU1 **possède** le compteur IEP
  (il appelle `IEP_INIT()`). Ne démarrez PAS l'ancien firmware `pru0_spindle`
  en même temps que `pru1_dual_stepper`.
- Les signaux ENABLE sont actifs LOW (0 = driver ON).

Rappel GPIO (PRU1 R30)
- P8_46  → SPINDLE_STEP (R30[1])
- P8_44  → SPINDLE_DIR  (R30[3])
- P8_42  → SPINDLE_EN   (R30[5]) (active-low)
- P8_45  → LATERAL_STEP (R30[0])
- P8_43  → LATERAL_DIR  (R30[2])
- P8_41  → LATERAL_EN   (R30[4]) (active-low)

1) Construire localement (sur le PC)

```bash
cd /home/nicolas/Documents/PlatformIO/Projects/PickupWinder/src/pru
make         # compile les firmwares (PRU0 + PRU1 dual-stepper)
make dtbo    # compile les .dtbo depuis dts/ (optionnel pour pinmux persistant)
```

2) Copier les fichiers sur la BBB (ex. vers /tmp)

```bash
scp build/am335x-pru1-fw debian@BBB_IP:/tmp/
scp build/dtbo/pickup-winder-p8-steppers.dtbo debian@BBB_IP:/tmp/   # si utilisé
scp test_spindle_p8.sh debian@BBB_IP:/tmp/
```

3) Installer sur la BBB (actions root)

```bash
# copier firmware + overlay + script
sudo cp /tmp/am335x-pru1-fw /lib/firmware/
sudo cp /tmp/pickup-winder-p8-steppers.dtbo /lib/firmware/   # optionnel
sudo cp /tmp/test_spindle_p8.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/test_spindle_p8.sh
```

4a) Méthode simple (recommandée pour tests rapides) — utiliser le script

```bash
sudo /usr/local/bin/test_spindle_p8.sh
```

Le script :
- met P8_41..P8_46 en `pruout` via `config-pin`
- arrête PRU1, assigne `am335x-pru1-fw` et démarre PRU1
- affiche l'état final

4b) Méthode manuelle (remoteproc + config-pin)

```bash
# config des pins (en local sur la BBB)
for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46; do
  sudo config-pin "$pin" pruout
done

# remoteproc (vérifier le numéro remoteproc sur votre kernel)
# remoteproc2 est habituel pour PRU1; adaptez si nécessaire
echo stop  | sudo tee /sys/class/remoteproc/remoteproc2/state
echo am335x-pru1-fw | sudo tee /sys/class/remoteproc/remoteproc2/firmware
echo start | sudo tee /sys/class/remoteproc/remoteproc2/state

# vérifier
cat /sys/class/remoteproc/remoteproc2/state
sudo dmesg | tail -20
```

5) Restaurer les pins (après test)

```bash
# via le script
sudo /usr/local/bin/test_spindle_p8.sh --restore

# ou manuellement
for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46; do
  sudo config-pin "$pin" default
done
```

6) Vérifications utiles
- `cat /sys/class/remoteproc/remoteproc2/state` → doit afficher `running`.
- `dmesg | tail -20` → messages kernel relatifs au chargement du firmware.
- Si vous avez des problèmes de démarrage, arrêtez PRU (`echo stop`) et relisez
  les logs : `dmesg`.

Notes additionnelles
- Pour un usage persistant, installez le `.dtbo` et ajoutez la surcharge
  via la méthode d'activation DT appropriée à votre distribution (ex: `uEnv.txt`
  ou `config-pin` au démarrage). La méthode exacte dépend de la version du
  kernel et des scripts de démarrage de la BBB.
- Ne lancez pas deux firmwares faisant chacun la consommation de la même
  `cmd_rhead`/anneau de commandes simultanément — cela corromprait le protocole.

Si vous voulez, je peux :
- ajouter un `make install`/`make deploy` plus sélectif (déployer seulement PRU1),
- automatiser le commit + push de `PRU_DEPLOY.md` (je peux le faire maintenant).
