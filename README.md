# Intro
This is a set of scripts to help store, visualize, and process data from our multisensor drone payload. 

# Setting up a new DVC data store
Create a new github repository and set it up locally if you wish. If you wish, you can add information to the README that describes the data that is going to be stored here.

Create a place to store the data. There are many options for this, as described [here](https://dvc.org/doc/command-reference/remote/add). An easy option is using Google Drive. Follow the instructions for your chosen method. In addition to setting the data store up for yourself, you may need to take steps to make this data accessible to your collaborators.

Now we need to add the scripts submodule. From the root of the project, run 
```
git submodule add git@github.com:russelldj/DVC_ROS_datastore_scripts.git  scripts
```
Now commit and push this new submodule.
# Adding data
I'd like to follow this naming convention:
`date/site_<site name>/collect_<number>`

Under this folder, you should create three folders: `raw`, `proccessed_1` and `processed_2`. Raw is for data that comes directly off the platform, which is primiarily rosbags. Processed 1 is for derived data which isn't useful to an end user, for example extracted images. Processesed 2 is for useful results, such as 3D models.

After placing data in the appropriate location, you can add it with the following commmands. Begin by ensuring that nothing is staged in your git workspace. Then use `dvc add -R <data folders>` to recursively add the files in the folders. The recursive option allows us to download individual files in the future, which is useful given the size of the data. This process will take some time. Once it is complete, run `dvc push` to upload the data to the default remote. In parellel, you can check your git staging area. You should see that `*.dvc` files pointing to the raw data files and `.gitignore` files ignoring the raw data have been automatically staged. Commit these changes and push them to github.

You can add default documentation using `python scripts/default_documentation.py`. Ideally, fill out the template, and then add, commit, and push these files to git.