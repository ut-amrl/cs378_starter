# cs378_starter

## Using this Repo
You will be using a copy of this repo for all development in the class. This repo provides starter code for the ROS subscriptions, publishers and control loops you will be using. Note that for the purposes of this course, this repository depends upon the `ut_automata` repository found here: https://github.com/ut-amrl/ut_automata. 

Please follow the instructions in the UT AUTOmata [reference manual](https://drive.google.com/file/d/1OUp6FGUPEClpTbXKK8XssCad9MBMnHP-/view?usp=sharing) before continuing.

### Duplicate the Repo
1. Make sure you're logged into your GitHub account.
2. Create a new repo with the same name under your GitHub account. Don't initialize that with anything, and **ensure that it is a private repository**.
3. `git clone <this repository url>` (found in the upper right)
4. `cd <cloned_repo>`
5. `git push --mirror <your new repository url>`

### Build the Repo
1. Make sure you are in the home directory of the repository.
2. Make sure you have added the path to the cloned directory in your `~/.profile`.
3. `make -j`
