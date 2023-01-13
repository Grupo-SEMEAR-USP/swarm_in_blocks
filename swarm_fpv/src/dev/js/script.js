const html_child = document.getElementById('blog') // retorna elemento html
console.log(html_child)

//const element = document.getElementsByClassName('one')
//console.log(element)


const tag_elem = document.getElementsByTagName('meta')
console.log(tag_elem) // retorna html collection

/////////
const element = document.querySelector('.one')
console.log(element) // querySelector pega o primeiro - retorna elemento

const element_all = document.querySelectorAll('.one')
console.log(element_all) // pega todos pelo seletor

const tag_seletor = document.querySelectorAll('[src]') //atributos src
console.log(tag_seletor) // este metodo retorna uma lista de nos (diferente de htmk collection)

tag_seletor.forEach(el => console.log(el)) // metodo unico de querySelectorAll/

///////////////////////////////

