# L104_IDP
This is the Github repository for the software components of this project

## To get the project locally
1. [Install Git](https://rogerdudler.github.io/git-guide/) if you haven't done so already
1. Press on the `Code` green button, then copy the url by clicking on the Clipboard
1. Open a `cmd` or `git bash` and type `git clone` then paste your URL and hit enter, this creates a local instance
   of the respository on your computer
   
## To edit & work on the project
1. Have a look at [this](https://rogerdudler.github.io/git-guide/), familiarise yourself with the idea of Git commits, branches, and pull requests
1. Before you start anything type 'git pull' into the command line of your IDE once you have the project open (or you can do this from the cmd,
   so long as you have navigated to the right folder)
1. Create a new branch, this is so that we don't break the stable 'master' branch. If you write some code that breaks things it's not detrimental because
   you did it on a seperate branch
2. To create the branch type 'git branch -b (branch name here)'
2. Ideally name the branch after the [trello](https://trello.com/b/TlyVqQNG/idp-l104) ticket that you are working on
1. Do some work and save it locally
1. Type 'git status' to see which files you've edited
1. ADD the files by using 'git add filename' (filename is the acutal filename, not the word filename)
2. Adding a file prepares it for a commit
2. Add all the files you've edited before committing
1. COMMIT your work, using 'git commit -m yourmessage' with a meaningful message please not just 'edited some stuff'
1. PUSH the work you did 'git push origin branch_name' (again not branch_name, but the name of your branch, 'git branch' will display all branches)
1. Your work will now be on github, if you want to merge it to the master branch open your branch and click the green button 'Create PR'
2. A PR or 'Pull Request' allows other team members to view and review your code, before it is then approved and pushed to the stable branch
1. If any of this still doesn't make any sense I'm happy to go through it with anyone

