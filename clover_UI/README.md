# Clover_UI

Our own homepage (Clover UI) for each user accessing the page through the apache localhost port, this homepage contains the main functionalities to launch a swarm or an individual drone such as Swarm Station and Blocks IDE.

<p align="center">
    <img src="homepage.gif" width=700>
</p>

For more info about `swarm_clover_blocks`, see the [offical documentation swarm in blocks](https://swarm-in-blocks.gitbook.io/swarm-in-blocks/introduction/swarm-in-blocks)

Internal package documentation is given below.

---

## Usability

This packages is completely available in the browser, so you can use it on any device with a browser through localhost. If you configured the Apache web service as our documentation suggests, you can (with the drone or the simulation connected) type "localhost" in the browser and acess our homepage web interface.

---

## How it works


### Organization

The package is organized into one main project, the CloverUI ReactJS project. From there you can find the Vite configuration to deploy the React project. Below we list the description of the organization of the package (it is very intuitive):

| Package | Description |
| ------- | -------- |
| `cloverUI`  |  The ReactJS project source|
| `cloverUI/index.html`  | One step of the ReactJS configuration, just calls the real project within the html file, but dont specific contains any real code for the webpage |
| `cloverUI/assets/`  | Nothing more thant the images that we use on the website |
| `cloverUI/src/`  | Source files page, here we can acess all  the code algorithms |
| `cloverUI/src/main.jsx`  | The second layer of callbacks for the project, it contains the call for the App and some arrow functions |
| `cloverUI/src/App.jsx`  | The heart of the project, here all the component pages are called (note that the specific code of each page are located in the Components folder) |
| `cloverUI/src/App.css`  | Styling general for the hole project |
| `cloverUI/src/index.css`  | Defining styling for the project (can or cannot be applied) |
| `cloverUI/src/style.js`  | Vite config por apllying the styles |
| `cloverUI/src/Componentes/`  | Folder with the real components that create the App itself |
| `cloverUI/src/Componentes/Bottom.jsx`  | ReactJS type of file for the Bottom of the Page (with all the links) |
| `cloverUI/src/Componentes/Cards.jsx`  | ReactJS type of file for the Cards (just a template) for the "Carrossel" |
| `cloverUI/src/Componentes/Carrossel.jsx`  | ReactJS type of file for the Carousel/Roundabout ("Carrossel" it is the Brazillian Portuguese word for this) of the Page that makes the slider effect |
| `cloverUI/src/Componentes/Clover_tools.jsx`  | ReactJS type of file for the Swarm Statin with the call to action |
| `cloverUI/src/Componentes/Easy2Swarm.jsx`  | ReactJS type of file for the Blocks IDE with the call to action |
| `cloverUI/src/Componentes/Frontpage.jsx`  | ReactJS type of file for the Front Page that is the firts contact of the user with our platform |
| `cloverUI/src/Componentes/Header.jsx`  | ReactJS type of file for the Header |
| `cloverUI/src/Componentes/styles/`  | Folder with the .css of each component |


### Frontend

We chose to use the ReactJS Framework to create the graphical user interface, as it is one of the most popular and with a very fast learning curve. In addition, it is very well documented and has a very active community, which facilitates the resolution of problems and doubts.

How he managed to make use of some more complex visual effects (Slider Carousel for example) and ensure a more user friendly platform. In addition, ReactJS is a relatively fast and easy tool to work with, which allowed us to create a more robust graphical interface with more functionality, since this page does not have its own Backend.

It is worth noting that we also use the ReactJS project builder, Vite, to facilitate project creation and ReactJS configuration. Vite is a ReactJS project builder that comes with ReactJS configured and ready to go.
