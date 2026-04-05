Test eQEP (BeagleBone)

But: Ce dossier fournit un test utilisateur pour lire un encodeur quadrature via l'interface eQEP du noyau BeagleBone.

Contenu:
- `read_eqep.py` : script Python autonome qui recherche un périphérique eQEP exposé via sysfs et affiche la position en boucle.
- `eqep_example.dts` : exemple de Device Tree overlay (modèle). Il sert de point de départ — adaptez les noms/pins selon votre carte et image.
- `Makefile` : cible `dtbo` pour compiler l'overlay (requiert `dtc`) et `install-overlay` pour copier le .dtbo dans `/lib/firmware` et tenter de l'activer (nécessite privilèges root et peut varier selon la distribution).

Usage rapide:

1) Vérifier si le noyau expose déjà eQEP:

   sudo find /sys -maxdepth 4 -type f -name position | grep -i eqep || true

   - Si vous voyez un chemin contenant `eqep` et un fichier `position`, vous pouvez lancer directement le script:

     python3 read_eqep.py --interval 0.1

2) Si aucun périphérique n'est trouvé, vous pouvez compiler l'overlay (exemple):

   make dtbo
   sudo make install-overlay

   Puis re-vérifier les fichiers sysfs (un `dmesg` peut montrer l'overlay chargé).

Notes de sécurité et compatibilité:
- Les Device Tree overlays et les chemins sysfs varient selon l'image Debian/Angstrom utilisée sur la BeagleBone.
- L'overlay fourni est un modèle et peut nécessiter des ajustements (numéros de fragment, pinmux, noms de périphériques). Testez prudemment.

Note sur les conflits avec le video overlay:
- Certaines images activent par défaut un overlay vidéo qui réserve des pins utilisés par HDMI ou la sortie LCD; si vous utilisez des pins proches du header vidéo vous devrez désactiver cet overlay pour libérer les broches. Sur les images Debian courantes, ajoutez/modifiez dans `/boot/uEnv.txt` (ou la configuration d'overlay de votre image) pour désactiver les overlays vidéo — par exemple enlever toute ligne `uboot_overlay_addrN=` qui référence un overlay vidéo, ou ne pas charger `BB-BONELT-HDMI`.

Si vous préférez que j'intègre la configuration pinmux (pull-up) directement dans l'overlay pour `P8_33`/`P8_35`, dites-le et je mets à jour `eqep_example.dts` (valeurs d'exemple déjà placées dans le fichier). Sinon activez temporairement les pull-ups via `config-pin`:

```bash
config-pin P8_33 gpio_pu
config-pin P8_35 gpio_pu
```

Mesure rapide: débranchez l'encodeur et mesurez la tension sur la broche; ~3.3V signifie qu'un pull-up est actif.

Si vous voulez, je peux adapter l'overlay aux pins/cape exacts de votre montage — dites-moi quels pins vous utilisez pour A/B/catalogue d'encodeur.