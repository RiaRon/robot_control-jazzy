# Existing Ready → Follow handoff

```mermaid
flowchart LR
    A[Ready A′ command] --> B[Ready completes]
    B --> C[Follow reads startup state]
    C --> D[RViz marker service snapshot]
    D --> E[Startup alignment]
    E --> F[Profile origin]
    C --> G[Command and IK seed]
    D --> H[Marker reference]
    H --> I[Profile begins after alignment]
    G --> I
```

기존 흐름은 Ready 종료 후 measured joint/TCP, marker, command와 IK continuity
reference를 하나의 명시적 handoff event로 기록하지 않았다. 따라서 stale marker나
startup 과도가 전체 성능 통계에 섞일 수 있었다.
