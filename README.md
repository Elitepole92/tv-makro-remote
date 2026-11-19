# TV-Makro-Fernbedienung (M5StickS3)

Tastendrücke einer beliebigen IR-Fernbedienung als **Makro aufnehmen** (mit den
Pausen dazwischen) und später per Knopfdruck am jeweiligen **TV-Modell** abspielen.
Gespeichert werden die Rohsignale, deshalb klappt das mit jeder TV-Marke.

## Erstes Flashen (einmalig per Laptop)

1. Arduino IDE, Boardpaket **M5Stack** (Boardverwalter-URL
   `https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json`),
   Bibliothek **M5Unified**.
2. `tv_makro_remote/tv_makro_remote.ino` öffnen (oder Inhalt in einen neuen Sketch kopieren).
3. Board **M5StickS3**, Port wählen, **Hochladen**.

## Bedienung am Stick

| Taste | kurz | lang |
|---|---|---|
| **A** (vorne) | Öffnen / Abspielen / Fertig | Optionen (neu aufnehmen, löschen) |
| **B** (Seite) | nächster Eintrag | vorheriger Eintrag |

Display geht nach 30 s aus, der erste Tastendruck weckt es nur.

**Aufnehmen:** TV-Modell öffnen → *Neues Makro aufnehmen* → Original-Fernbedienung
aus 20–30 cm auf die **Oberseite** des Sticks richten, Tasten in Ruhe nacheinander
drücken → **A** = fertig.

**Browser (optional):** *WLAN / Browser* → am Handy ins WLAN `TV-Remote-xxxx`
(Passwort `fernbedienung`) → http://192.168.4.1 → umbenennen, sortieren, löschen.

## Updates ohne Laptop

1. Einmalig im Browser (siehe oben) unter **Updates** das eigene WLAN eintragen,
   z.B. den iPhone-Hotspot (am iPhone *Maximale Kompatibilität* einschalten).
2. Hotspot an → am Stick **Update** wählen. Er lädt die neueste Version von GitHub
   und startet neu. Makros und TV-Modelle bleiben erhalten.

Im Code stehen keine Zugangsdaten; das WLAN-Passwort liegt nur auf dem Stick.

## Neue Version veröffentlichen

Bei jedem Push auf `main` baut GitHub Actions die Firmware und stellt sie als
Release bereit (`firmware.bin` + `version.txt`). Die sichtbare Versionsnummer steht
in `tv_makro_remote/VERSION`, verglichen wird die fortlaufende Build-Nummer.

Die `.ino` wird aus `werkzeug/vorlage.ino` + `werkzeug/web.html` erzeugt:

```bash
python3 werkzeug/erzeugen.py werkzeug/vorlage.ino werkzeug/web.html tv_makro_remote/tv_makro_remote.ino
```
