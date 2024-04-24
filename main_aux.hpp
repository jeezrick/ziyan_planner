#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

template <typename T> void static inline saveArray(T* in_array, size_t length, std::string filepath)
{
    std::ofstream ouF;
    ouF.open(filepath.c_str(), std::ofstream::binary);
    ouF.write(reinterpret_cast<const char*>(in_array), sizeof(T) * (length));
    ouF.close();
    printf("Save file: %s successful.\n", filepath.c_str());
}

template <typename T>
void static inline printArr(T* arr, int x, int y)
{
    for (int i = 0; i < y; i++)
    {
        for (int j = 0; j < x; j++)
        {
            std::cout << ((int)arr[i * x + j]) << ",";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
}

void readArray(std::string filepath, void* buffer, std::streamsize size)
{
    std::ifstream fin(filepath, std::ios::binary | std::ios::in);
    if (!fin)
    {
        throw std::runtime_error("Cannot open the file");
    }

    fin.read(static_cast<char*>(buffer), size);
    if (!fin)
    {
        throw std::runtime_error("Error reading file");
    }

    fin.close();
    std::cout << "Load file: " << filepath << " successful." << std::endl;
}

void convertArray(uint8_t* dst, uint8_t* src, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        uint8_t value = src[i]; 
        if (value == 1) {
            value = 254; // 将值为 1 的元素修改为 100
        }
        dst[i] = value; // 将修改后的值存回数组中
    }
}
