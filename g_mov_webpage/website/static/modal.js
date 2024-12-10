// Obtener el modal y la imagen dentro del modal
var modal = document.getElementById("customModal"); // Selecciona el elemento con el ID 'customModal'
var modalImg = document.getElementById("modalImage"); // Selecciona el elemento de imagen dentro del modal con el ID 'modalImage'

// Obtener las imágenes que abren el modal
var img1 = document.getElementById("openModal1"); // Selecciona la primera imagen que abrirá el modal
var img2 = document.getElementById("openModal2"); // Selecciona la segunda imagen que abrirá el modal
var img3 = document.getElementById("openModal3"); // Selecciona la tercera imagen que abrirá el modal

// Obtener el botón de cierre del modal
var span = document.getElementsByClassName("close")[0]; // Selecciona el primer elemento con la clase 'close'

// Función para abrir el modal y mostrar la imagen correspondiente
function openModal(imgElement) {
  modal.style.display = "flex"; // Cambia el estilo de display del modal a 'flex' para mostrarlo y centrar su contenido
  modalImg.src = imgElement.src; // Actualiza la fuente de la imagen dentro del modal con la fuente de la imagen clicada
}

// Asignar la función 'openModal' al evento 'onclick' de cada imagen
img1.onclick = function() {
  openModal(this); // 'this' se refiere a la imagen que fue clicada
}

img2.onclick = function() {
  openModal(this); // 'this' se refiere a la imagen que fue clicada
}

img3.onclick = function() {
  openModal(this); // 'this' se refiere a la imagen que fue clicada
}

// Asignar una función al botón de cierre para cerrar el modal
span.onclick = function() {
  modal.style.display = "none"; // Cambia el estilo de display del modal a 'none' para ocultarlo
}

// Asignar una función al objeto 'window' para cerrar el modal cuando se haga clic fuera del contenido del modal
window.onclick = function(event) {
  if (event.target == modal) { // Verifica si el elemento clicado es el modal mismo (fondo oscuro)
    modal.style.display = "none"; // Cambia el estilo de display del modal a 'none' para ocultarlo
  }
}

