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

## Bedienung am Stick (hochkant)

| Taste | kurz | lang |
|---|---|---|
| **A** (vorne) | Öffnen / Abspielen / Fertig | Optionen (neu aufnehmen, löschen) |
| **B** (Seite) | nächster Eintrag | **Menü**: WLAN/Browser, Update, IR-Test, Ausschalten (Copyright + Version) |
| **Power** | ganz aus – bleibt aus, bis Power wieder gedrückt wird | |

**Aufnehmen:** TV-Modell öffnen → *Aufnehmen* → Original-Fernbedienung
aus 20–30 cm auf die **Oberseite** des Sticks richten, Tasten in Ruhe nacheinander
drücken → **A** = fertig.

**IR-Test:** B lang → *IR-Test* zeigt live, was der Empfänger sieht: gültige Signale
und ignorierte Störungen (mit Anzahl Flanken und Pulslängen). Störungen ohne
Fernbedienung deuten auf Störquellen hin (Lampen, Sonne, Elektronik in der Nähe).

**Browser (optional):** B lang → *WLAN/Browser* → am Handy ins WLAN `TV-Remote-xxxx`
(Passwort `fernbedienung`) → http://192.168.4.1 → umbenennen, sortieren, löschen.
Das Stick-WLAN geht von selbst aus, wenn 5 min niemand verbunden ist.

## Strom sparen

- Prozessor läuft mit 80 MHz statt 240 MHz (nur fürs Update kurz schneller).
- Nach **10 s** wird das Display dunkler, nach **30 s** schläft der Stick **tief**
  (Display, Prozessor und Funk aus). Taste A oder B weckt ihn; der erste Druck
  weckt nur, danach steht er beim zuletzt benutzten Makro.
- **Power-Taste** = ganz aus. Am USB-Kabel lässt er sich nicht ganz abschalten,
  dann geht er in den Tiefschlaf.
- WLAN ist nur an, wenn man es braucht (Browser, Update).

## Updates ohne Laptop

1. Einmalig im Browser (siehe oben) unter **Updates** das eigene WLAN eintragen,
   z.B. den iPhone-Hotspot (am iPhone *Maximale Kompatibilität* einschalten).
2. Hotspot an → am Stick B lang → **Update**. Er lädt die neueste Version von GitHub
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
