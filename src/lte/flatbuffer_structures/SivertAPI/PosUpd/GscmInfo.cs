// <auto-generated>
//  automatically generated by the FlatBuffers compiler, do not modify
// </auto-generated>

namespace SivertAPI.PosUpd
{

using global::System;
using global::FlatBuffers;

public struct GscmInfo : IFlatbufferObject
{
  private Struct __p;
  public ByteBuffer ByteBuffer { get { return __p.bb; } }
  public void __init(int _i, ByteBuffer _bb) { __p.bb_pos = _i; __p.bb = _bb; }
  public GscmInfo __assign(int _i, ByteBuffer _bb) { __init(_i, _bb); return this; }

  public int Tx { get { return __p.bb.GetInt(__p.bb_pos + 0); } }
  public int Rx { get { return __p.bb.GetInt(__p.bb_pos + 4); } }
  public double Rss { get { return __p.bb.GetDouble(__p.bb_pos + 8); } }

  public static Offset<GscmInfo> CreateGscmInfo(FlatBufferBuilder builder, int Tx, int Rx, double Rss) {
    builder.Prep(8, 16);
    builder.PutDouble(Rss);
    builder.PutInt(Rx);
    builder.PutInt(Tx);
    return new Offset<GscmInfo>(builder.Offset);
  }
};


}
