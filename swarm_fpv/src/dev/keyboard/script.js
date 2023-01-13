//document.addEventListener('keydown', (event) => {
    //console.log(event.__proto__)
 //   var name = event.key;
//    var code = event.code;

    //console.log(`tecla pressionada: ${name} com codigo: ${code} . ${event.repeat}`)
//    if (code == "KeyA" && event.repeat == false) {
//        console.log("tecla a pressionada")
//    }
    //alert(`Tecla pressionada ${name} \r\n Codigo: ${code}`);
//})

let object1Size = {
    width: 20,
    height: 20
};

let position = {
    x: 10,
    y: 10
};
let moveRate = 10;
let growthRate = 2;
let object1 = document.getElementsByClassName('object1');


function updateYPosition(distance) {
    position.y = position.y - distance;

    //condiçoes de borda
    if (position.y < 0) {
        position.y = 499;

    } else if (position.y > 499) {
        position.y=0;
    }
}

function updateXPosition(distance) {
    position.x = position.x + distance;

    //condiçoes de borda
    if (position.x < 0) {
        position.x = 499;

    } else if (position.x > 499) {
        position.x=0;
    }
}

function updateSize(growth) {
    object1Size.width += growth;
    object1Size.height += growth;
}


function refreshPosition() {
    let x = position.x - (object1Size.width/2);
    let y = position.y - (object1Size.height/2);

    let transform = "translate(" + x + " " + y + ")";
    console.log(transform)

    object1[0].setAttribute("transform", transform);
    object1[1].setAttribute("transform", transform)
    //object1.setAttribute("transform", "rotate(" + x + ")" );
}

function refreshSize() {
    let newSize = object1Size.height;
    //let newSize
    object1[0].setAttribute("width", newSize);
    object1[0].setAttribute("height", newSize);

}

window.addEventListener("keydown", function(event) {
    if (event.defaultPrevented) {
      return;
    }
    if (event.code === "ArrowDown"){
        // Handle "down"
        updateYPosition(-moveRate);
    } else if (event.code === "ArrowUp"){
        // Handle "up"
        updateYPosition(moveRate);
    } else if (event.code === "ArrowLeft"){
        // Handle "left"
        updateXPosition(-moveRate);
    } else if (event.code === "ArrowRight"){
        // Handle "right"
        updateXPosition(moveRate);
    } else if (event.code === "KeyW") {
        updateSize(growthRate)
    } else if (event.code === "KeyS") {
        updateSize(-growthRate)
    }
    
    refreshPosition();
    refreshSize();
    event.preventDefault();
  }, true);

  