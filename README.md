# Noise-Cancellation_DSP-Labor-2024-2025-

Im Rahmen der Vorlesung "Digitale Signalprozessoren" wurde unserer Gruppe die Aufgabe "Noise Cancellation" zugeteilt.
Wir haben uns dazu entschieden einen Notch-Filter, einen LMS-Filter und einen Wienerfilter zu implementieren. Diese sind adaptiv und werden
je nach Rauschen abgewechselt.

Unsere aktuelle Implementierung besteht aus folgenden Bausteinen:

1. Einlesen der DMA Puffer Werte in einen zirkulären Puffer
2. FFT-Trafo der Eingangsdaten (STFT)
3. Analyse des Rauschens in den den ersten Zyklen der Aufnahme (centroid,spread)
   -> Bestimmung der Rauschart (sinusartig, breitbandig z.B. weißes Rauschen)
5. Berechnung der Filter Koeffizienten
6. Anwendung der Filter auf das folgende Signal
7. IFFT
8. Ausgabe über einen zirkulären Puffer 

Was bereits funktioniert:

Der LMS-Filter und der Notch-Filter funktionieren sehr gut in Echtzeit. Wenn also zunächst z.B. ein Sinus eingespeißt wird und daraufhin 
gesprochen wird, wird der Sinus komplett herraus gefiltert. Man kann den Sinus während dessen auch hoch pitchen, wodurch er nicht mehr gefiltert wird.
Nachdem der uC resettet wird, werden neue Filterkoeffizienten bestimmt und der hochgepitchte Sinus wird ebenfalls gefilert.









