///////////////////////////////////////////////////////////////////////////////
///
/// \brief TelemetryRecorder Implementation.
///
//////////////////////////////////////////////////////////////////////////////

#include <iomanip>

#include "exo_simu/core/TelemetryRecorder.h"


namespace exo_simu
{
    TelemetryRecorder::TelemetryRecorder(std::shared_ptr<TelemetryData const> const & telemetryDataInstance) :
    telemetryData_(telemetryDataInstance),
    startTime_(0.0), 
    flows_(),
    recordedBytesLimits_(0),
    recordedBytesDataLine_(0),
    recordedBytes_(0),
    headerSize_(0), 
    integersAddress_(), 
    integerSectionSize_(0), 
    floatsAddress_(), 
    floatSectionSize_(0)
    {
        // Empty.
    }

    TelemetryRecorder::~TelemetryRecorder()
    {
        flows_.back().close();
    }

    result_t TelemetryRecorder::initialize(void)
    {
        result_t returnCode = result_t::SUCCESS;

        // Get telemetry data infos.
        telemetryData_->getData(integersAddress_,
                                integerSectionSize_, 
                                floatsAddress_, 
                                floatSectionSize_);
        recordedBytesDataLine_ = sizeof(float64_t) + integerSectionSize_ + floatSectionSize_;

        // Get the header
        std::vector<char_t> header;
        telemetryData_->formatHeader(header);
        headerSize_ = header.size() / sizeof(uint8_t);

        // Create a new MemoryDevice and open it.
        flows_.clear();
        returnCode = createNewChunk();

        // Write the Header
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = flows_[0].write(header);
            recordedBytes_ = headerSize_;
        }

        if (returnCode == result_t::SUCCESS)
        {
        }

        return returnCode;
    }

    result_t TelemetryRecorder::createNewChunk()
    {
        result_t returnCode = result_t::SUCCESS;

        // Close the current MemoryDevice, if any and if it was opened.
        if (!flows_.empty())
        {
            flows_.back().close();
        }
        
        // Create a new one.
        uint32_t isHeaderThere = flows_.empty();
        uint32_t maxRecordedDataLines = ((MAX_BUFFER_SIZE - isHeaderThere*headerSize_) / recordedBytesDataLine_);
        recordedBytesLimits_ = isHeaderThere*headerSize_ + maxRecordedDataLines * recordedBytesDataLine_;
        flows_.emplace_back(recordedBytesLimits_);
        returnCode = flows_.back().open(OpenMode::READ_WRITE);

        if (returnCode == result_t::SUCCESS)
        {
            recordedBytes_ = 0;
        }
        else
        {
            std::cout << "Error - TelemetryRecorder::createNewChunk - Impossible to create a new chunk of memory buffer." << std::endl;
            returnCode = result_t::ERROR_GENERIC;
        }
        
        return returnCode;
    }

    result_t TelemetryRecorder::flushDataSnapshot(float64_t const & timestamp)
    {
        result_t returnCode = result_t::SUCCESS;

        if (recordedBytes_ == recordedBytesLimits_)
        {
            returnCode = createNewChunk();
        }

        if (returnCode == result_t::SUCCESS)
        {
            // Write time
            flows_.back().write(timestamp);

            // Write data, integers first.
            flows_.back().write(reinterpret_cast<uint8_t const*>(integersAddress_), integerSectionSize_);

            // Write data, floats last.
            flows_.back().write(reinterpret_cast<uint8_t const*>(floatsAddress_), floatSectionSize_);

            // Update internal counter
            recordedBytes_ += sizeof(float64_t) + integerSectionSize_ + floatSectionSize_;
        }

        return returnCode;

    }

    void TelemetryRecorder::getData(std::vector<std::string>             & header, 
                                    std::vector<float64_t>               & timestamps, 
                                    std::vector<std::vector<int32_t> >   & intData, 
                                    std::vector<std::vector<float32_t> > & floatData)
    {
        header.clear();
        timestamps.clear();
        intData.clear();
        floatData.clear();

        float64_t timestamp;
        std::vector<int32_t> intDataLine;
        intDataLine.resize(integerSectionSize_ * sizeof(uint8_t) / sizeof(int32_t));
        std::vector<float32_t> floatDataLine;
        floatDataLine.resize(floatSectionSize_ * sizeof(uint8_t) / sizeof(float32_t));
        
        for (uint32_t i=0; i<flows_.size(); i++)
        {
            int64_t pos_old = flows_[i].pos();
            flows_[i].seek(0);

            if (i == 0)
            {
                std::vector<char_t> headerCharBuffer;
                headerCharBuffer.resize(headerSize_ * sizeof(uint8_t) / sizeof(char_t));
                flows_[i].readData(reinterpret_cast<uint8_t *>(headerCharBuffer.data()), headerSize_);
                char_t const * pHeader = &headerCharBuffer[0];
                uint32_t posHeader = 0;
                std::string fieldHeader(pHeader);
                while (posHeader < headerCharBuffer.size() && fieldHeader.size())
                {
                    header.emplace_back(std::move(fieldHeader));
                    posHeader += header.back().size() + 1;
                    fieldHeader = std::string(pHeader + posHeader);
                }
            }

            uint32_t numberLines = (pos_old - flows_[i].pos()) / recordedBytesDataLine_;
            timestamps.reserve(timestamps.size() + numberLines);
            intData.reserve(intData.size() + numberLines);
            floatData.reserve(floatData.size() + numberLines);
            for (uint32_t j=0; j < numberLines; j++)
            {
                flows_[i].readData(reinterpret_cast<uint8_t *>(&timestamp), sizeof(float64_t));
                flows_[i].readData(reinterpret_cast<uint8_t *>(intDataLine.data()), integerSectionSize_);
                flows_[i].readData(reinterpret_cast<uint8_t *>(floatDataLine.data()), floatSectionSize_);
                timestamps.emplace_back(timestamp);
                intData.emplace_back(intDataLine);
                floatData.emplace_back(floatDataLine);
            }

            if (i == flows_.size() - 1)
            {
                flows_[i].seek(pos_old);
            }
        }
    }
}