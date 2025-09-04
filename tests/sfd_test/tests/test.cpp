#include <grayscale_sdf.h>
#include "threadpool.h"
#include <SFML/Graphics.hpp>
#include <stdlib.h>
#include <chrono>

ThreadPoolHandle tpHdl;

errno_t launchTask(sdf_task task__, sdf_taskHandle hdl__) {
	ThreadPoolTaskHandle* hdl = (ThreadPoolTaskHandle*)hdl__;
	ThreadPoolTask task = { task__.args, task__.func };
	return ThreadPool_LaunchTask(tpHdl, task,hdl);
}

errno_t joinTask(sdf_taskHandle hdl) {
	ThreadPool_JoinTask((ThreadPoolTaskHandle*)hdl);
	return 0;
}

int main(int argc, const char* const*)
{
	tpHdl.threadCount = 2;
	if (ThreadPool_New(&tpHdl, 15000))
		return EXIT_FAILURE;

	sf::RenderWindow window(sf::VideoMode({ 900, 500 }), "GRAYSCALE SDF");
	sf::Image image;

	if (!image.loadFromFile("assets/a.png"))
		return EXIT_FAILURE;

	sf::Texture texture(image);
	sf::Sprite sprite(texture);

	sdf_instance instance;
	instance.malloc = malloc;
	instance.free = free;
	instance.threadCount = tpHdl.threadCount;
	instance.launchTask = (sdf_launchTask_fptr)launchTask;
	instance.joinTask = (sdf_joinTask_fptr)joinTask;
	instance.taskHandleSize = sizeof(ThreadPoolTaskHandle);

	sdf_threshold threshold;
	threshold.range = { .type = SDF_RANGE, .channel = SDF_CHANNEL_A, .lowerBound = 0.5f, .upperBound = 1.f };

	sdf_imageInfo imgInfo = {
		.pixels = image.getPixelsPtr(),
		.width = (uint16_t)image.getSize().x,
		.height = (uint16_t)image.getSize().y,
		.format = SDF_FORMAT_R8G8B8A8
	};

	auto start = std::chrono::high_resolution_clock::now();

	uint32_t distanceFieldSize=0;
	if (sdf_imageToSdf(&instance, &imgInfo, 45, &threshold, 1, &distanceFieldSize, SDF_FORMAT_R8, nullptr))
		return EXIT_FAILURE;

	int8_t* sdf = (int8_t*)malloc(distanceFieldSize);
	if (sdf_imageToSdf(&instance, &imgInfo, 45, &threshold, 1, nullptr, SDF_FORMAT_R8, (int8_t*)sdf))
		return EXIT_FAILURE;

	auto end = std::chrono::high_resolution_clock::now();
	float durationMS = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


	printf("SDF Evaluated in %f MS. Thread count: %d\n", durationMS, instance.threadCount);

	sf::Image sdfImage;
	sdfImage.resize({imgInfo.width, imgInfo.height});

	for (uint32_t y = 0; y < imgInfo.height; ++y) {
		for (uint32_t x = 0; x < imgInfo.width; ++x) {
			const uint8_t stride = 1;
			int8_t v = (int8_t)sdf[((y * imgInfo.width) + x) * stride];
			float f = fabsf((float)v / 127);
			int8_t rad = sdf[((y * imgInfo.width) + x) * stride + 1];
			rad = 0;
			sf::Color c;
			if (v > 0) {
				c = { 0,(uint8_t)rad,(uint8_t)v,255 };
			}
			else {
				c = { (uint8_t)(-v),(uint8_t)rad,0,255 };
			}
			//c = { 0,(uint8_t)rad,(uint8_t)v,255 };
			sdfImage.setPixel({ x,y },c);
		}
	}
	/*
	const float lineLength = 5.0f;  
	const int step = 4;                
	sf::VertexArray lines(sf::PrimitiveType::Lines);
	for (uint32_t y = 0; y < imgInfo.height; y += step) {
		for (uint32_t x = 0; x < imgInfo.width; x += step) {
			int8_t rad = sdf[((y * imgInfo.width) + x) * 2 + 1];
			float dir = ((float)rad / 255.0f) * 2.0f * 3.14159265f;

			sf::Vector2f start((float)x + 300, (float)y); // +300 offset if SDF sprite is at 300
			sf::Vector2f end = start + sf::Vector2f(cosf(dir), sinf(dir)) * lineLength;

			lines.append(sf::Vertex(start, sf::Color::Red));
			lines.append(sf::Vertex(end, sf::Color::Red));
		}
	}
	*/

	sf::Texture sdfTexture(sdfImage);
	sf::Sprite sdfSprite(sdfTexture);
	sdfSprite.setPosition({ 300,0 });

	while (window.isOpen()) {
		while (const std::optional event = window.pollEvent())
		{
			if (event->is<sf::Event::Closed>())
				window.close();
		}

		window.clear(sf::Color::Green);
		window.draw(sprite);
		window.draw(sdfSprite);
		//window.draw(lines);
		window.display();
	}

	ThreadPool_Destroy(&tpHdl);
	return EXIT_SUCCESS;
}