# Battery Scheduler for Autonomous Mobile Robots

# Prerequisits
1. MongoDB, 'message_store' database with collection of 'task_events'.
2. PRISM - https://github.com/bfalacerda/prism
3. Clone of this repository with the files in the folder models. 

# Modifications
1. The 'path' variables have to be modified in the following files and are indicated by ##SPECIFY LOCATION## wherever possible.
2. PRISM path should be modified.
  


Once the data is set up, and the paths have been specified. 1. generate sample rewards 2. Run testing_wo_rhc, with no_days set to 1 for single day policies. Places that require modification in path will be indicated in as many places as possible with #######SPECIFY LOCATION#######

Rewards
1. Add task_events to mongodb.
2. Update path on read_tasks file, based on access of collection in mongodb
3. Use rewards_uncertain_hk.py to change the number of days used for testing, and to select days for generating sample rewards. The change should be made in the initialisation, under validation == True. Also change the number of days required.
4. generate_sample.py generates the sample rewards and writes into the file 'sample_rewards'. Please edit the path for this file. Plans are made using these sample rewards.

Battery
1. Add battery data. 
2. battery_model.py, forms yaml files for charging and discharging. Specify locations for creation of these files. The locations of the battery data, from which model is learnt is specified when the class is instantiated from other modules. 

Probabilistic rewards and battery model
1. testing_wo_rhc.py - specify the number of days to be tested, location of prism files, and location where plan has to be written to. Calls functions for learning models, solving the models in prism.
2. Make sure you have the properties file in place.
3. form_prism_script.py writes the prism script from the learnt models. Path should be modified.
4. prism_simulate.py reads the 'optimal' adversary and forms the plan based on the genrated sample rewards. Path should be modified.

Receding Horizon Control
Paths must be mofified. 
rhc.py calls all the sub-functions. rhc_prism_script.py writes the prism model, rhc_prism_parse reads the adversary

Reduced Recding Horizon Control
Paths must be mofified. 
rrhc.py calls all the sub-functions. reduced_rhc_script.py writes the prism model, rrhc_prism_parse reads the adversary



