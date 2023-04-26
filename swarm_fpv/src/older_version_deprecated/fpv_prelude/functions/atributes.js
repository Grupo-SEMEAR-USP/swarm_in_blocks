// var id = 5
var list = [3, 6, 60]
// var element = document.getElementById('dropdown')
// var child = document.createElement('option')
// child.textContent = `Clover ${id}`
// child.setAttribute('value', `${id}`)
// element.appendChild(child)

// element.firstElementChild.setAttribute('option', '5')
// element.firstElementChild.

function addIds() {
    var element = document.getElementById('dropdown')
    
    for (let id in list) {
      // create_element(lista[x])
      var child = document.createElement('option')
      console.log(id)
      child.textContent = `Clover ${list[id]}`
      child.setAttribute('value', `${list[id]}`)
      element.appendChild(child)
    }
  }
  
  addIds()
  