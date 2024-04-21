#include <iostream>
#include <random>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <cassert>

// CountPointVariables //
static int in{};
static int total{};
static std::mutex mutex{};
static std::atomic<int> inAtomic{};
static std::atomic<int> totalAtomic{};

// Other Variables //
static const unsigned int s_NrThreads{ std::thread::hardware_concurrency() };
static constexpr unsigned int s_NrSamples{ 100'000'000 };
static constexpr unsigned int s_NrMeasurements{ 10 };

struct Vector2f
{
    float x;
    float y;
};

struct MeasureData
{
    unsigned int time;
    float result;
};

class MeasureHelper final
{
public:
    MeasureHelper(unsigned int capacity)
    {
        m_Data.reserve(capacity);
    }
    ~MeasureHelper() = default;

    MeasureData GetAverageMeasure()
    {
        // Check
        if (m_Data.size() < 2)
        {
            assert(false);
            return MeasureData{};
        }

        // Get Max and Min value
        auto minmax = std::minmax_element(m_Data.begin(), m_Data.end(),
            [](const MeasureData& dataA, const MeasureData& dataB)
            {
                return dataA.time < dataB.time;
            }
        );

        // Remove Max and Min value
        m_Data.erase(std::remove_if(m_Data.begin(), m_Data.end(), 
            [&](const MeasureData& data)
            {
               return data.time == minmax.first->time || data.time == minmax.second->time;
            }
        ), m_Data.end());

        // Get Average
        unsigned int sumTime{};
        float sumResult{};
        unsigned int count{};
        for (const auto& data : m_Data)
        {
            sumTime += data.time;
            sumResult += data.result;
            ++count;
        }
        const unsigned int averageTime{ sumTime / count };
        const float averageResult{ sumResult / count };

        return MeasureData{ averageTime, averageResult };
    }

    void AddData(const MeasureData& data)
    {
        m_Data.emplace_back(data);
    }

private:
    std::vector<MeasureData> m_Data;
};

Vector2f GetRandomPoint()
{
    thread_local std::random_device rd{};
    thread_local std::default_random_engine eng{ rd() };
    thread_local std::uniform_real_distribution<float> distr{ -1, 1 };
    return { distr(eng), distr(eng) };
}

void CountPoints(int number)
{
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++in;
        }
    }
    total += number;
}

void CountPointsMutex(int number)
{
    int localIn{};
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++localIn;
        }
    }

    std::lock_guard<std::mutex> lock(mutex);
    in += localIn;
    total += number;
}

void CountPointsAtomic(int number)
{
    for (int idx{}; idx < number; ++idx)
    {
        const Vector2f point{ GetRandomPoint() };
        if (point.x * point.x + point.y * point.y < 1.f)
        {
            ++inAtomic;
        }
    }
    totalAtomic += number;
}

void PrintTime(const std::string& version, int milisec, float result)
{
    std::cout << version << " version: " << milisec << " ms\n";
    std::cout << "result: " << result << "\n\n";
}

void RunOriginalVersion()
{
    MeasureHelper helper{ s_NrMeasurements };

    std::chrono::high_resolution_clock::time_point start{};
    std::chrono::high_resolution_clock::time_point end{};
    std::chrono::duration<double, std::milli> diff{};

    in = 0;
    total = 0;

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        start = std::chrono::high_resolution_clock::now();
        CountPoints(s_NrSamples);
        end = std::chrono::high_resolution_clock::now();
        diff = end - start;
        helper.AddData(MeasureData{static_cast<unsigned int>(diff.count()), 4.f * in / total });
        in = 0;
        total = 0;
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Original", measuredData.time, measuredData.result);
}

void RunMutexVersion()
{
    const unsigned int pointsPerThread{ s_NrSamples / s_NrThreads };
    MeasureHelper helper{ s_NrMeasurements };

    std::vector<std::jthread> threads{};
    std::chrono::high_resolution_clock::time_point start{};
    std::chrono::high_resolution_clock::time_point end{};
    std::chrono::duration<double, std::milli> diff{};

    in = 0;
    total = 0;

    for (unsigned int measureNr{}; measureNr < s_NrMeasurements; ++measureNr)
    {
        threads.clear();
        threads.resize(s_NrThreads);
        start = std::chrono::high_resolution_clock::now();
        for (auto& thread : threads)
        {
            thread = std::jthread(CountPointsMutex, pointsPerThread);
        }
        for (auto& thread : threads)
        {
            thread.join();
        }
        end = std::chrono::high_resolution_clock::now();
        diff = end - start;
        helper.AddData(MeasureData{ static_cast<unsigned int>(diff.count()), 4.f * in / total });
        in = 0;
        total = 0;
    }

    MeasureData measuredData{ helper.GetAverageMeasure() };
    PrintTime("Mutex", measuredData.time, measuredData.result);
}

void RunAtomicVersion()
{
    const unsigned int pointsPerThread{ s_NrSamples / s_NrThreads };

    std::vector<std::jthread> threads{};
    std::chrono::high_resolution_clock::time_point start{};
    std::chrono::high_resolution_clock::time_point end{};
    std::chrono::duration<double, std::milli> diff{};

    threads.clear();
    threads.resize(s_NrThreads);
    start = std::chrono::high_resolution_clock::now();
    for (auto& thread : threads)
    {
        thread = std::jthread(CountPointsAtomic, pointsPerThread);
    }
    for (auto& thread : threads)
    {
        thread.join();
    }
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    PrintTime("Atomic", static_cast<int>(diff.count()), 4.f * inAtomic / totalAtomic);
    inAtomic = 0;
    totalAtomic = 0;
}

int main()
{
    // Print Info
    std::cout << "# threads: " << s_NrThreads << "\n";
    std::cout << "# samples: " << s_NrSamples << "\n";
    std::cout << "# measurements: " << s_NrMeasurements << "\n";
    std::cout << "\n";

    // Original Version //
    RunOriginalVersion();

    // Mutex Version //
    RunMutexVersion();

    // Atomic Version //
    RunAtomicVersion();

    // Promise - Future Version //


    return 0;
}