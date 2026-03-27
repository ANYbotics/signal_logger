# Log-File Format

## Naming Convention

Log files are written to the current working directory. In ROS setups that is
typically `/home/$USER/.ros/`.

```text
silo_#y#m#d_#H-#M-#S_#NR.silo
```

Where:

- `#y`: four-digit year
- `#m`: two-digit month
- `#d`: two-digit day
- `#H`: two-digit hour
- `#M`: two-digit minute
- `#S`: two-digit second
- `#NR`: five-digit running counter

Example:

```text
silo_20160913_12-13-49_00113.silo
```

## File Structure

### Header

The file begins with a warning not to edit the file in a text editor, followed
by metadata such as:

```text
# Log File: silo_20160913_12-13-49_00113.silo
# Time synchronization offset:
27
# Number of Log Elements:
6
```

The element list then describes the layout of the binary payload:

```text
# (Element Name) (Data Size In Bytes) (No Data Points) (Divider) (Buffer looping (1 or 0)) (Data Type)
/myLoggerNamespace/time_s 8 100 1 0
/myLoggerNamespace/time_ns 8 100 1 0
/myLoggerNamespace/myGroup1/myDataA 1 50 2 0 uint8
/myLoggerNamespace/myGroup1/myDataB 8 50 1 1 double
/myLoggerNamespace/myGroup2/myDataC 4 10 5 0 single
/myLoggerNamespace/myGroup2/myDataD 2 25 4 1 int16
```

Notes:

- `myDataA` stores 1 byte per sample with divider 2, so 100 collects result in
  50 stored samples.
- `myDataB` stores 8 bytes per sample with divider 1 and uses a looping buffer.
- `myDataC` stores 4 bytes per sample with divider 5.
- `myDataD` stores 2 bytes per sample with divider 4 and uses a looping buffer.

This header is sufficient to parse the binary payload without losing
information.

### Binary Data

The payload starts after:

```text
# Binary Data
```

The binary data is written in the same order as the header entries. For the
example above:

- the first 800 bytes are `time_s`
- the second 800 bytes are `time_ns`
- the next 50 bytes are `myDataA`
- the next 400 bytes are `myDataB`
- the next 40 bytes are `myDataC`
- the last 50 bytes are `myDataD`

## Matlab Support

A Matlab helper script for parsing and plotting these files is available in
`signal_logger_std/matlab/processData.m`.
