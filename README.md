
### Simulator starten ###

Damit der Simulator startet müssen die Dateien parcour.txt und stepinfo.txt im aktuellen Verzeichnis liegen.
Der Simulator wartet beim Starten auf eine IPC Verbindung. D.h. der ImageViewer muss ebenfalls gestartet werden.


### stepinfo.txt ###
Der Simulator benötigt zusätzliche Informationen über die von euch programmierten Schritte. Diese werden in der Datei stepinfo.txt eingestellt.


### parcour.txt ###
Der Parcour, den der Roboter durchlaufen muss ist konfigurierbar. Er lässt sich in parcour.txt einstellen.

### Optionen ###
In Simulator.cpp Zeile 45..47 werden die Optionen für den Simulator festgelegt. Hier kann eingestellt werden ob der Roboter links oder rechts startet.
Außerdem können hier die Pfade zur parcour.txt und stepinfo.txt geändert werden.
