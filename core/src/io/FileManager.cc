#include <map>
#include <algorithm>

#include "exo_simu/core/io/FileManager.h"


namespace exo_simu
{
namespace core
{
    FileManager::FileManager()
        : files_()
    {
    }


    ManagedFile::ManagedFile(FileDevice* file, AbstractIODevice* firstClient)
        : file_(file)
        , clients_()
    {
        if (firstClient != nullptr)
        {
            clients_.push_back(firstClient);
        }
    }


    ManagedFile::ManagedFile(ManagedFile&& other)
        : file_(nullptr)
        , clients_()
    {
        file_ = other.file_;
        other.file_ = nullptr;
        std::swap(clients_, other.clients_);
    }


    ManagedFile& ManagedFile::operator=(ManagedFile&& other)
    {
        file_ = other.file_;
        other.file_ = nullptr;
        std::swap(clients_, other.clients_);
        return *this;
    }


    ManagedFile::~ManagedFile()
    {
        if (file_ != nullptr)
        {
            file_->close();
            delete file_;
        }

        for (AbstractIODevice* io : clients_)
        {
            io->close();
        }
    }


    bool_t ManagedFile::operator==(std::string const& filename)
    {
        return file_->name() == filename;
    }


    std::vector<ManagedFile>::iterator FileManager::find(std::string const& name, std::vector<ManagedFile>& files)
    {
        return std::find(files.begin(), files.end(), name);
    }


    hresult_t FileManager::processNewClient(AbstractIODevice* client)
    {
        struct protocol::Handshake startHeader;
        hresult_t const rc = client->read(startHeader);
        if (failed(rc))
        {
            return rc;
        }

        if (startHeader.version != protocol::version::V1)
        {
            (void) client->write(error::E_ENOTSUP); // Don't care of the return code: the upper layer will close the communication.
            return error::E_ENOTSUP;
        }

        // Check if someone has already opened this file, then add the client to the pool.
        auto it = find(startHeader.filename, files_);
        if (it != files_.end())
        {
            it->clients_.push_back(client);
            return client->write(error::S_OK);
        }

        // File does not exist yet: create and open it.
        ManagedFile entry(createFile(startHeader.filename), client);
        files_.push_back(std::move(entry));
        hresult_t const rcOpen = files_.back().file_->open(startHeader.modes);

        // Report result.
        hresult_t rcWrite = client->write(rcOpen);
        if (succeeded(rcOpen))
        {
            return rcWrite;
        }

        return rcOpen;
    }


    void FileManager::processFiles()
    {
        // Aggregated clients - Erase–remove idiom.
        files_.erase(std::remove_if(files_.begin(), files_.end(),
                                    [&](ManagedFile & managedFile) -> bool_t
                                    {
                                        return this->processFile(managedFile);
                                    }),
                     files_.end());
    }


    bool_t FileManager::processClients(AbstractIODevice* io, std::map<uint64_t, std::vector<std::vector<uint8_t>>> & entries)
    {
         // We need at least the timestamp and the buffer size to proceed.
        int32_t const minimalEntrySize = sizeof(struct protocol::FrameHeader);

        int32_t bytesToRead = static_cast<int32_t>(io->bytesAvailable());
        if (bytesToRead < 0)
        {
            // client disconnected.
            io->close();
            return true;
        }

        while (minimalEntrySize < bytesToRead)
        {
            struct protocol::FrameHeader frame;
            std::vector<uint8_t> payload;

            hresult_t rc = io->read(frame);
            if (succeeded(rc))
            {
                bytesToRead -= sizeof(frame);

                switch (frame.cmd)
                {
                    case protocol::command::WRITE:
                    {
                        payload.resize(frame.dataSize);
                        rc = io->read(payload);

                        if (succeeded(rc))
                        {
                            // Update bytesToRead.
                            bytesToRead -= payload.size();

                            // std::move to avoid copy.
                            entries[frame.timestamp].push_back(std::move(payload));
                        }

                        break;
                    }

                    case protocol::command::CLOSE:
                    {
                        // Asked to be closed.
                        io->close();
                        return true;
                    }

                    case protocol::command::GET_POS:
                    case protocol::command::SET_POS:
                    case protocol::command::READ:
                    case protocol::command::GET_SIZE:
                    default:
                    {
                        rc = error::E_ENOTSUP;
                        break;
                    }
                }
            }

            if (failed(rc))
            {
                // Error reading: client disconnection.
                io->close();
                return true;
            }
        }

        // Erase–remove idiom -> false means 'do not remove the entry'.
        return false;
    }


    bool_t FileManager::processFile(ManagedFile& managedFile)
    {
        // Map to reorder entries thanks to timestamps.
        std::map<uint64_t, std::vector<std::vector<uint8_t>> > entries;

        // Erase–remove idiom.
        managedFile.clients_.erase(std::remove_if(managedFile.clients_.begin(), managedFile.clients_.end(),
                                                  [&](AbstractIODevice * io) -> bool_t
                                                  {
                                                      return this->processClients(io, entries);
                                                  }),
                                   managedFile.clients_.end());

        // Write entries
        for (auto entriesIt = entries.begin(); entriesIt != entries.end(); ++entriesIt)
        {
            for (std::vector<uint8_t> const& entry : entriesIt->second)
            {
                managedFile.file_->write(entry);
            }
        }

        // No more clients on this file: close file and destroy entry.
        if (managedFile.clients_.empty())
        {
            return true;
        }

        // Erase–remove idiom -> false means 'do not remove the entry'.
        return false;
    }


    FileDevice* FileManager::createFile(std::string const& filename)
    {
        return new FileDevice(filename);
    }
}
}
