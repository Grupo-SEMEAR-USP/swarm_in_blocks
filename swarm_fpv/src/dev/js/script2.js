const element = document.querySelector('h1')

element.textContent = "eita a"
element.innerText = "bb"

element.innerHTML = "mudança <li>bold</li>"

const element2 = document.querySelector('input')
console.log(element2.value) 

// manipulando atributos

const header = document.querySelector('header')
header.setAttribute('id', 'header')

const headerID = document.querySelector('#header') // selector entende # como id
console.log(headerID)
console.log(headerID.getAttribute('class'))

/////
const element_body = document.querySelector('body')
//element_body.style.backgroundColor = 'green'
element_body.classList.add('active', 'green')
element_body.classList.toggle('active')
console.log(element_body.classList)

function print() {
    console.log("clicou")
}

function changeTitle() {
    document.querySelector('#title').innerText = "mudou o titulo"
}

const input = document.querySelector('.ata')

input.onkeydown = () => {
    console.log("aaaa")
}