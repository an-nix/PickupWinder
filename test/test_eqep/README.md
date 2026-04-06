# test_eqep — Encodeur quadrature via eQEP1 (BeagleBone Black, kernel 6.12)

## Contenu

| Fichier | Rôle |
|---------|------|
| `eqep_example.dts` | Overlay Device Tree pour eQEP1 sur P8_33/P8_35 |
| `Makefile` | Compile le `.dtbo` via `dtc` |
| `read_eqep.py` | Lit la position de l'encodeur via le sysfs counter framework |
| `check_eqep_env.sh` | Script de diagnostic (dmesg, show-pins, sysfs) |

---

## Matériel

| BBB Pin | Pad AM335x | Offset pinmux | Fonction | Raccordement |
|---------|-----------|---------------|----------|--------------|
| **P8_35** | conf_mcasp0_ahclkr | `0x0D0` | EQEP1A_in (MODE4) | Encodeur canal A |
| **P8_33** | conf_mcasp0_fsr | `0x0D4` | EQEP1B_in (MODE4) | Encodeur canal B |
| P8_1 ou P9_1 | — | — | GND | Masse encodeur |

Encodeur 2 phases uniquement (pas d'index/strobe nécessaire).
Pull-up internes activés dans l'overlay (résistance externe non requise).

Adresse matérielle eQEP1 : `epwmss1` @ `0x48302000`, counter child @ `0x48302180`.

---

## Découverte critique : format 3 cellules (kernel 6.12)

Le nœud pinmux du BBB sous kernel 6.12 déclare **`#pinctrl-cells = <2>`**.
Cela impose le format **3 cellules par pin** dans `pinctrl-single,pins` :

```
<pad_offset   config_flags   mux_mode>
```

Le driver écrit `config_flags | mux_mode` dans le registre pad.

> ⚠️ L'ancien format 2 cellules `<offset value>` ne configure que le premier pin.
> Le second pin est interprété comme un offset sans valeur associée et ignoré
> silencieusement. C'est ce qui causait « P8_35 ok, P8_33 reste en gpio ».

### Preuves dans le DTB actif (kernel 6.12.76-bone50)

```
pinmux@800 {
    #pinctrl-cells = <0x02>;            ← 3 cellules/pin dans pinctrl-single,pins
    pinctrl-single,register-width = <0x20>;
    pinctrl-single,function-mask = <0x7f>;

    i2c0-pins {
        /* <offset  config  mux>  × 2 pins = 6 valeurs */
        pinctrl-single,pins = <0x188 0x30 0x00  0x18c 0x30 0x00>;
    };
    i2c2-pins {
        pinctrl-single,pins = <0x178 0x30 0x03  0x17c 0x30 0x03>;
    };
    user-leds-s0-pins {
        /* 4 pins × 3 cellules = 12 valeurs */
        pinctrl-single,pins = <0x54 0x00 0x07  0x58 0x10 0x07
                               0x5c 0x00 0x07  0x60 0x10 0x07>;
    };
    /* Ancien overlay BBORG — format 2 cellules (pré-6.12, ne pas copier) */
    pinmux_comms_can_pins {
        pinctrl-single,pins = <0x184 0x32  0x180 0x12>;
    };
};
```

### Signification des bits de config_flags (AM335x)

| Bit | Nom | 0 | 1 |
|-----|-----|---|---|
| 6 | SLEWCTRL | rapide | lent |
| 5 | RXACTIVE | entrée désactivée | entrée activée |
| 4 | PUTYPESEL | pull-down | pull-up |
| 3 | PUDEN | pull **activé** | pull désactivé |
| 2:0 | MUXMODE | — | 0–7 |

| config_flags | Signification | Usage |
|---|---|---|
| `0x30` | fast, rx-ON, pull-UP, pull-EN | Entrée numérique avec pull-up (eQEP, encodeur) |
| `0x08` | fast, rx-OFF, pull-DISABLED | Sortie PRU (STEP/DIR/EN) |
| `0x10` | fast, rx-OFF, pull-UP, pull-EN | Sortie GPIO avec pull-up |
| `0x00` | fast, rx-OFF, pull-DOWN, pull-EN | Sortie GPIO avec pull-down |

Formule : registre écrit = `config_flags | mux_mode`
Exemple eQEP : `0x30 | 0x04 = 0x34`

---

## Compilation et déploiement

```bash
# Sur le host (x86)
make                          # → BB-EQEP-EXAMPLE-00A0.dtbo
scp BB-EQEP-EXAMPLE-00A0.dtbo beagle@192.168.6.2:/home/beagle/transfer/

# Sur la BBB
sudo cp /home/beagle/transfer/BB-EQEP-EXAMPLE-00A0.dtbo /lib/firmware/
```

Dans `/boot/uEnv.txt` sur la BBB, ajouter ou remplacer une ligne `uboot_overlay_addrN` libre :

```
uboot_overlay_addr4=/lib/firmware/BB-EQEP-EXAMPLE-00A0.dtbo
```

Puis redémarrer.

---

## Vérification après reboot

```bash
# Les deux pins doivent être réclamés par counter@48302180
show-pins | grep -E 'P8\.(33|35)'
# Attendu :
# P8.35 ... 4 ... counter@48302180 (pinctrl_eqep1_pins)
# P8.33 ... 4 ... counter@48302180 (pinctrl_eqep1_pins)

# counter0 doit être présent
ls /sys/bus/counter/devices/

# Compter les fichiers count
find /sys/bus/counter/devices -name count

# dmesg eQEP
dmesg | grep -i 'eqep\|counter\|epwmss'
```

---

## Lecture de l'encodeur

```bash
# Lecture en boucle (Ctrl+C pour arrêter)
python3 read_eqep.py

# Lecture unique
python3 read_eqep.py --once

# Intervalle personnalisé (secondes)
python3 read_eqep.py --interval 0.05
```

Le script cherche automatiquement le bon chemin sysfs :
- Counter framework (kernel 6.x) : `/sys/bus/counter/devices/counter*/count*/count`
- Legacy (kernel 4.x) : `/sys/devices/**/eqep*/position`

---

## Diagnostic si les pins ne s'appliquent pas

```bash
# Lire la valeur actuelle des registres pad (BBB)
sudo devmem2 0x44E100D0 w   # P8_35 — doit lire 0x34
sudo devmem2 0x44E100D4 w   # P8_33 — doit lire 0x34

# Forcer manuellement (sans reboot, test uniquement)
sudo devmem2 0x44E100D0 w 0x34
sudo devmem2 0x44E100D4 w 0x34
show-pins | grep -E 'P8\.(33|35)'

# Vérifier que l'overlay est bien chargé par U-Boot
dmesg | grep -i 'overlay\|BB-EQEP'
cat /proc/device-tree/__symbols__/pinctrl_eqep1_pins 2>/dev/null && echo FOUND
```

---

## Erreurs courantes

| Symptôme | Cause probable | Remède |
|----------|---------------|--------|
| P8_33 reste en gpio 7, P8_35 ok | Format 2-cell dans l'overlay (ancien format) | Passer au format 3-cell `<offset config mux>` |
| Aucun `counter0` dans sysfs | `epwmss1` ou `eqep1` pas activé | Vérifier `status = "okay"` sur les deux nœuds |
| `counter0` présent mais position toujours 0 | Pins pas en MODE4 | Vérifier avec `devmem2` que les registres valent 0x34 |
| Reboot ne démarre plus | Syntaxe DTS invalide | Retirer la ligne `uboot_overlay_addrN` dans `uEnv.txt` et reflasher |
| `fragment@ {}` dans le DTS | Syntaxe obsolète, fragile sur kernel 6.12 | Remplacer par `&node {}` direct |