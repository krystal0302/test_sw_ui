# Introduction

The artifact CLI tool for FARobot

# Usage

* Command format: `python3 artifact_sdk_cli.py [cmd] [subcmd] [argument]...`
  - 3 kinds of cmd: type, wrapper, artifact
  - `python3 artifact_sdk_cli.py -h`

* type: `python3 artifact_sdk_cli.py type -h`
```bash
# List type
python3 artifact_sdk_cli.py type list
```

* wrapper: `python3 artifact_sdk_cli.py wrapper -h`
```bash
# List wrapper
python3 artifact_sdk_cli.py wrapper list
# Create wrapper
python3 artifact_sdk_cli.py wrapper create mywrapper wrapper_type
# Delete wrapper
python3 artifact_sdk_cli.py wrapper delete mywrapper
# List service
python3 artifact_sdk_cli.py wrapper list_service mywrapper
# Get code from service
python3 artifact_sdk_cli.py wrapper get_service mywrapper myservice
# Save code to service
python3 artifact_sdk_cli.py wrapper save_service mywrapper myservice python_code
```

# Code Architecture

* artifact_sdk_cli.py: Main program
* types: Here is the type list to create wrapper.
* wrappers: When you create new wrapper, it'll be put in this folder
