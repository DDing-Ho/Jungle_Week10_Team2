#pragma once
#include "DynamicVertexBuffer.h"
#include "IndexBuffer.h"
#include "VertexTypes.h"
#include "Render/RHI/D3D11/Device/D3DDevice.h"

class FDynamicMeshBuffer
{
public:
    FDynamicMeshBuffer() = default;
    ~FDynamicMeshBuffer() { Release(); }

    FDynamicMeshBuffer(const FDynamicMeshBuffer&) = delete;
    FDynamicMeshBuffer& operator=(const FDynamicMeshBuffer&) = delete;
    FDynamicMeshBuffer(FDynamicMeshBuffer&&) = default;
    FDynamicMeshBuffer& operator=(FDynamicMeshBuffer&&) = default;

    template <typename VertexType>
    void Create(ID3D11Device* InDevice, const TMeshData<VertexType>& InMeshData)
    {
        Release();
        if (InMeshData.Vertices.empty())
        {
            return;
        }

        uint32 VertexCount = static_cast<uint32>(InMeshData.Vertices.size());
        uint32 VertexByteWidth = VertexCount * sizeof(VertexType);
        DynamicVertexBuffer.Create(InDevice, VertexCount, sizeof(VertexType));

        if (!InMeshData.Indices.empty())
        {
            uint32 IndexCount = static_cast<uint32>(InMeshData.Indices.size());
            uint32 IndexByteWidth = IndexCount * sizeof(uint32);
            IndexBuffer.Create(InDevice, InMeshData.Indices.data(), IndexCount, IndexByteWidth);
        }
    }

    template <typename VertexType>
    void UpdateVertices(ID3D11DeviceContext* InContext, const TMeshData<VertexType>& InMeshData)
    {
        DynamicVertexBuffer.Update(InContext, InMeshData.Vertices.data(), InMeshData.Vertices.size());
    }

    void Release();

    FDynamicVertexBuffer& GetVertexBuffer() { return DynamicVertexBuffer; }
    FIndexBuffer& GetIndexBuffer() { return IndexBuffer; }
    const FDynamicVertexBuffer& GetVertexBuffer() const { return DynamicVertexBuffer; }
    const FIndexBuffer& GetIndexBuffer() const { return IndexBuffer; }

    bool IsValid() const { return DynamicVertexBuffer.GetBuffer() != nullptr; }

private:
    FDynamicVertexBuffer DynamicVertexBuffer;
    FIndexBuffer         IndexBuffer;
};