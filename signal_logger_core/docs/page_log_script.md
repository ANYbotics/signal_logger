# Logger Script Format

The logger script lists all registered logger elements and can optionally
override their default configuration. The file format is YAML.

Example:

```yaml
log_elements:
  - name: /myLoggerNamespace/myGroup1/myDataA
    enabled: true
    divider: 1
    buffer:
      type: 0
      size: 5
    action: 0
  - name: /myLoggerNamespace/myGroup1/myDataB
    enabled: false
    buffer:
      type: 1
      size: 50
    action: 1
  - name: /myLoggerNamespace/myGroup2/myDataC
    enabled: true
    divider: 5
    action: 2
  - name: /myLoggerNamespace/myGroup2/myDataD
    enabled: false
    divider: 10
    buffer:
      type: 1
      size: 100
```

`buffer.type` stores the integer value of `signal_logger::BufferType`. The same
applies to `action`, which stores `signal_logger::LogElementAction`.
