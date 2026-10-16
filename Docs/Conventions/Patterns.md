



## Collaborator
The Subject_Collaborator_* Collaborators follow, verb last.
When coupling two peers:
0 coordination points → no wrapper, no bridge. Callers compose directly.
≤ ~5 coordination points, peers are public vocabulary → Multi-context free functions (Subject_Collaborator_*). No wrapper type.
Many coordination points, or composite owns meaningful state → Mediator wrapper with peer getters (Option A).
Peers must be hidden from callers → Full-forwarder facade (Option B).

