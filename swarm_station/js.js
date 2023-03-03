const openModalLand = document.querySelector("#open-land");
const closeModalLand = document.querySelector(".close-land");
const fade_land = document.querySelector("#fade_land");

const land = document.querySelector("#land");

const toggleModalLand = () => {
	land.classList.toggle("hide");
	fade_land.classList.toggle("hide");
};

[openModalLand, closeModalLand, fade_land].forEach((el) => {
	el.addEventListener("click", () => toggleModalLand());
});




const openModalSeg = document.querySelector(".open-seg");
const closeModalSeg = document.querySelector(".close-seg");
const fade_seg = document.querySelector("#fade_seg");

const seg = document.querySelector("#seg");

const toggleModalSeg = () => {
	seg.classList.toggle("hide");
	fade_seg.classList.toggle("hide");
};

[openModalSeg, closeModalSeg, fade_seg].forEach((el) => {
	el.addEventListener("click", () => toggleModalSeg());
});

$(document).ready(function() {
	$(".desc").hide();
	$('input[type="radio"]').click(function() {
		var test = $(this).val();
		$(".desc").hide();
		$("#"+test).show();
	});
});

function show(){
	document.getElementById('swarm').classList.toggle('active');
	document.getElementById('clover').classList.toggle('active');
	document.getElementById('open-land').classList.toggle('active');
}


