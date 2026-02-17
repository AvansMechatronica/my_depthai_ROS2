# AI Netwerk


## Training

### Yolo V5(voorkeur)
[Yolo V5 training](https://colab.research.google.com/drive/1g6glENMju05OKoRe5LVrZOE8yrH-ALjX#scrollTo=aRxrsCBzIb1I)

### Yolo V8(ongetest)
[Yolo V8 training](https://colab.research.google.com/drive/1M_Fnquk3dmfuBLfpjgMGVwj35Em1-wfP)

Na training dient het netwerkbestand `best.pt` geconverteerd te worden naar een tweetal DepthAI compatible bestanden:
* `best.superblob`
** Eigenlijke AI Netwerk

* `config.json`

** JSON bestand met karakteristieke eigenschappen van het `blob` bestand zoals labels van de te detecteren objecten.

Gebruik daarvoor de volgende converter:

[DepthAI Converter](https://docs.luxonis.com/cloud/hubai/quick-conversion/)

## Uitrollen

```