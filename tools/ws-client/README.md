# PickupWinder Python WebSocket client

Client léger pour se connecter à `ws://<device>/ws`, lancer l'auto-calibration et récupérer les données de télémétrie.

## Prérequis

- Python 3.8+
- Installer le client WebSocket Python :

```bash
pip install websocket-client
```

## Usage

```bash
python tools/ws-client/py_ws_client.py --host 192.168.4.1 --calib --rpm 80 --repeats 3 --apply --log data.csv
```

Options importantes :

- `--host` : adresse IP du pickup winder.
- `--port` : port HTTP (80 par défaut).
- `--calib` : déclenche `auto_calib` à la connexion.
- `--rpm` : RPM de calibration.
- `--repeats` : nombre de traversées pour calibration.
- `--apply` : applique le décalage proposé au `turnsPerPass`.
- `--log` : enregistre CSV avec champs temps/turns/rpm/hz/latPos/activeTpp/latScale etc.

## Sortie

- Log console : ligne CSV par message WS reçue.
- fichier CSV si `--log` est suffisant pour tracer rapidement via Excel / Python.
