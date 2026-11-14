# StateMachine

Hierarchical state machine: const `State_T` nodes in ROM, one leaf pointer in RAM per instance.

- [Architecture.md](Architecture.md): the model, the tree walk, inputs, threads and the lock, path ids, adding a state.
- Public API [StateMachine.h](StateMachine.h); node definition [State.h](State.h), [State_PathId.h](State_PathId.h).
- Engine: [_StateMachine_Tree.c](_StateMachine_Tree.c), hierarchical, the one in use; [_StateMachine.c](_StateMachine.c), flat, `Reset` only today; [_StateMachine_Lock.h](_StateMachine_Lock.h); [_State_Node.c](_State_Node.c).
- [StateMachine_Thread.h](StateMachine_Thread.h), proc compositions, available, not used; [Extension/](Extension/), templates behind their own enables.
- Open thread decisions: [0010 shared word writes](../../Docs/Decisions/0010_SharedWordWrites.md), [0011 lock nesting](../../Docs/Decisions/0011_StateMachineLockNesting.md).
