# Assignment2-PathFinding Setup

## Overview
1. [Accepting the Assignment](#Accepting-the-Assignment)
2. [Setting up the Environment](#Setting-up-the-Environment)
3. [Coding style](#coding-style)

## Instructions
> [!WARNING]
> These instructions assume you are working with a Unix-like operating system. For Windows-based machines, you must ensure the proper use of directory slashes; i.e., using `\` instead of `/` where appropriate.

### Accepting the Assignment

- **Using Git**: We assume you are comfortable using the Git distributed version control system and the Github web service. If you need to refresh your Git-related skills, we recommend you check out the many tutorials available online, including (but not limited to):
    - [Learn how to use Git](https://githowto.com)
    - [Learn how to use Git with Github](https://github.com/skills/introduction-to-github)
- **Using Github Classroom**: In order to organize assignment submissions, we will be using Github Classroom: an online tool to create and manage digital classrooms and assignments. For your convenience, we have created this guide to walk you through the steps required to obtain and submit your assignments.

> [!NOTE]
> The Github classroom screenshots you see below were created for a previous class I have taught, the Fall 2023 edition of CS 5360 | CS 6360: Virtual Reality. While the screenshots will not show you the correct course content, you should see the same screens and modal boxes. If you have any issues seeing the content, please do not hesitate to reach out!

![Screenshot of Github Classroom Login](/README/githubclassroom.png)
1. After you click on the Github Classroom Assignment Link provided on Canvas, you will be required to sign in to your Github account. If you have not created one, please do so and then return to this screen.

![Screenshot of the Github Classroom Modal to confirm joining a Github Classroom](/README/jointheclassroom.png)
2. Once you have signed in, you will be required to associate your Github account with an identifier. We have created identifiers based on your uNID; find yours in the list, click on it, and confirm the link.

![Screenshot of the Accept this assignment screen within Github Classroom](/README/acceptassignment.png)
3. Associating your Github account to an identifier allows you to accept assignments within Github Classroom. If you used the link on Canvas, you should be prompted to accept `Assignment 1: Movement AI`. You should see a screen *similar* to the above, but remember, the above screenshot is for a different class. After you see this prompt, click on `Accept this assignment`.

![Screenshot of the Assignment Accepted screen within Github Classroom](/README/assignmentaccepted.png)
4. Accepting the assignment prompts Github Classroom to create a custom and private assignment repository for you. After a few minutes, refresh the page in order to see the details of your custom repository. You can also see the details via the main Classroom portal.

![Screenshot of the Assignment Repository Created screen within Github Classroom](/README/assignmentrepocreated.png)
5. Github Classroom names your custom and private assignment repository using a standard format. Once created, you may click on the link to see your repository, as illustrated in the next step.

![Screenshot of the repository created in the previous step within your own Github account](/README/assignmentrepo.png)
6. Once created, your assignment repository functions just like a regular Github repository. To submit assignments, you must ensure the repository reflects the submission content by the assignment deadline.

- **Cloning the Github Classroom Repository**: After you have created your custom and private assignment repository via Github Classroom, clone it to your local machine within the directory of your choice to get started.

Your Assignment 1 repository should have been created with the following standard format: 
```
https://github.com/UtahGameAIClass/[assignment-name]-[your Github username]. 
```
For example, for the username `@recardona`, the assignment repository should be cloneable via the following bash terminal command:
```git
git clone https://github.com/UtahGameAIClass/assignment-1-movement-AI-recardona
```

> [!CAUTION]
> Remember to swap the URL to your assignment repository URL on Github.

### Setting up the Environment

##### How we set up this repository
This repository contains a very lightly modified version of the [openFrameworks Github Repository](https://github.com/openframeworks/openFrameworks) repository that allows you to install openFrameworks from Github. This is how we produced the repository:
1. Per [openFrameworks INSTALL_FROM_GITHUB.md](https://github.com/openframeworks/openFrameworks/blob/master/INSTALL_FROM_GITHUB.md), we cloned the `master` branch of openFrameworks as follows:
```git
git clone --recursive https://github.com/openframeworks/openFrameworks --depth 1
```
2. We created the top-level directory `meta`.
3. We moved the following files into `meta`:
    - [CHANGELOG.md](meta/CHANGELOG.md) 
    - [CODE_OF_CONDUCT.md](meta/CODE_OF_CONDUCT.md) 
    - [CONTRIBUTING.md](meta/CONTRIBUTING.md) 
    - [LICENSE.md](meta/LICENSE.md) 
    - [README.md](meta/README.md) 
    - [SECURITY.md](meta/SECURITY.md) 
    - [THANKS.md](meta/THANKS.md)
4. We created the top-level directory `README` for the images referenced in the README/SETUP files, and of course, this file itself.

##### How to configure openFrameworks on your machine
The repository is set up with the intent of you following [the instructions that openFrameworks provides for installing from Github](https://github.com/openframeworks/openFrameworks/blob/master/INSTALL_FROM_GITHUB.md). Strictly speaking there are two ways for you to set up openFrameworks, but if you have never had experience installing a program from a command line terminal, we *strongly* encourage you to follow Option 2:
- **Option 1 (Expert Users)**: Follow all of the relevant instructions in the file [INSTALL_FROM_GITHUB.md](./INSTALL_FROM_GITHUB.md). It has been left at the root level of the project deliberately, so that all relative path references remain correct.
- **Option 2**: Go to [https://openframeworks.cc/download/](https://openframeworks.cc/download/) and download the proper version for your operating system/development environment.
    1. When you select the version of openFrameworks that's right for you, you will download a compressed archive that begins with the prefix `of_v0.12.0_`.
    2. Uncompress the archive and open it. You should observe that the folder structure of the unarchived folder will look very similar to the structure of this repository. 
    3. You are going to replace several folders within this repository with folders from your newly downloaded/unarchived folder that begins with `of_v0.12.0_`.
        1. Copy the following folders from the `of_v0.12.0_` folder:
            - `addons/`
            - `docs/`
            - `examples/`
            - `libs/`
            - `projectGenerator/`
            - `scripts/`
        2. Paste the folders into the root-level of this repository; replace any old folders with new ones in the copy.

> [!CAUTION]
> The instructor can confirm that it is possible to set this up entirely throught the Terminal on the MacOS operating system. Your mileage may vary, but Google is your friend here.

##### Setting up your development environment on your machine
Each major operating system (macOS, Windows, Linux) has a corresponding development environment that's recommended for setup, which you can observe on the [openFrameworks Download Page](https://openframeworks.cc/download/). 

For this class you may use whatever development environment you prefer, but the general structure of this repository must be preserved (this is in order to facilitate grading). Further, your submission must be placed in the folder [myApps](./apps/myApps/) subfolder. Your openFrameworks app can be called anything, but if you're looking for a suggestion: name it `gameAI` (original, we know).

> [!IMPORTANT]
> The recommendeded way to create an openFrameworks application is via the [Project Generator](https://openframeworks.cc/learning/01_basics/create_a_new_project/), which will auto-create the folder in the correct location within your openFrameworks install.

> [!NOTE]
> The instructor has set up a development environment on macOS using [VS Code](https://openframeworks.cc/setup/vscode/) and using [Homebrew](https://brew.sh)'s version of the [GNU compiler collection (gcc)](https://formulae.brew.sh/formula/gcc).  

### Coding style 
While you are not required to use any coding style, we *strongly* suggest that you follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for your code. The easiest way to do it is to install a [linter](https://en.wikipedia.org/wiki/Lint_(software)) that is configured to follow it and highlight/auto-fix when you deviate from it.

The reason we suggest following a style is to help us debug things during office hours/virtual grading as the need arises. Help us help you!
