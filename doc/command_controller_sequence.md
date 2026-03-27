# CommandController sequence diagram

This Mermaid sequence diagram shows when and how `CommandController` receives and dispatches commands from WebSocket and UART, and how listeners consume them.

```mermaid
sequenceDiagram
    autonumber
    participant WS as "WebInterface (WebSocket)"
    participant UART as "LinkSerial (UART)"
    participant MAIN as "main() loop"
    participant CMD as "CommandController"
    participant SESSION as "SessionController"
    participant WINDER as "WinderApp"

    Note over WS,UART: Incoming commands originate from WebSocket or UART display

    WS->>WS: _onWsEvent(data)
    WS-->>CMD: registeredCallback
    UART->>UART: poll parses frame
    UART-->>CMD: parsedCommand

    MAIN->>CMD: process buffer
    Note right of CMD: pop one command from ring buffer
    CMD->>SESSION: listener1.handleCommand
    alt SESSION consumes
        SESSION-->>CMD: return true
    else not consumed
        SESSION-->>CMD: return false
        CMD->>WINDER: listener2.handleCommand
        WINDER-->>CMD: return true
    end
    CMD-->>MAIN: processing done

    Note over MAIN,WINDER: WinderApp or SessionController may change state, trigger status updates
    MAIN->>WS: web.sendUpdate(status)
    MAIN->>UART: serialLink.sendStatus
```
