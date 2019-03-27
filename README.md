# Battery Scheduler for Autonomous Mobile Robots

## Prerequisits
1. MongoDB, 'message_store' database with collection of 'task_events'.
2. PRISM - https://github.com/bfalacerda/prism
3. Clone of this repository with the files in the folder models. 

## Modifications
1. The 'path' variables have to be modified in the following files and are indicated by ##SPECIFY LOCATION## wherever possible.
2. Paths to battery files should be modified.
2. PRISM path should be modified.
3. Choose any array of test dates, from the possible dates.
  
## Running the files
1. Run fhc.py for finite horizon control.
2. Run rhc.py for receding horizon control.
3. Run rule_based_\*.py for rule based controls.
