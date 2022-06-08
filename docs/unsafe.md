If a function has the tag `unsafe` in its name, this means to call it, you must ensure a precondition, which is found in
the doc comment. In RELEASE mode, we won't enforce this condition, so if you fail to meet it, this is undefined
behaviour. In DEBUG mode, we enforce this condition. Meaning an assertion fault is triggered if you fail to meet the
precondition.