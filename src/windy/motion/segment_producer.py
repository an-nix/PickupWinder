"""Protocole SegmentProducer.

Tout générateur satisfaisant ce protocole peut être utilisé comme source de
mouvement par MultiAxisRampStreamer.

Le homing est intentionnellement exclu : HomingMove est un CompositeMove
exécuté phase par phase par MoveQueue, pas via SegmentProducer.
"""
from __future__ import annotations

from typing import Iterator, Protocol

from transport.messages import MultiAxisSegment


class SegmentProducer(Protocol):
    """Produit un flux fini de MultiAxisSegment avec des numéros de séquence croissants.

    Implémenteurs existants :
      - StepProfileSegmentGenerator  (rampe trapézoïdale, utilisé par RampMove)
      - SynchronizedSegmentGenerator (engrenage électronique, utilisé par WoundMove)
    """

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        ...