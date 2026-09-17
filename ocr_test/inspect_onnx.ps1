# 解析 ONNX 的 ModelProto，打印 graph 的输入/输出张量名与形状。
# ONNX 就是 protobuf：ModelProto.graph 是 field 7，ir_version 是 field 1。
param([string]$Path = 'D:\vs26\CUADC_fixedwing\PP-OCRv5_mobile_rec.onnx')

$bytes = [System.IO.File]::ReadAllBytes($Path)

function Read-Varint {
    param([byte[]]$B, [ref]$I)
    $shift = 0; $val = 0L
    while ($true) {
        if ($I.Value -ge $B.Length) { throw "varint 越界" }
        $c = $B[$I.Value]; $I.Value++
        $val = $val -bor ([long]($c -band 0x7F) -shl $shift)
        if (($c -band 0x80) -eq 0) { break }
        $shift += 7
        if ($shift -gt 63) { throw "varint 过长" }
    }
    return $val
}

function Skip-Field {
    param([byte[]]$B, [ref]$I, [int]$Wire)
    switch ($Wire) {
        0 { Read-Varint $B $I | Out-Null }
        1 { $I.Value += 8 }
        2 { $len = [int](Read-Varint $B $I); $I.Value += $len }
        5 { $I.Value += 4 }
        default { throw "不支持的 wire type $Wire" }
    }
}

# TensorProto: name=8, dims=1(repeated int64), dim_val=2(repeated Dimension), data_type=2? 
# 实测本模型把 data_type 放在 field 2 会冲突，故下面按 wire type 分派：
#   field 1 wire0 -> dims
#   field 2 wire2 -> Dimension 子消息 (dim_value/dim_param)
#   field 2 wire0 -> data_type  (部分导出器如此)
function Read-TensorProto {
    param([byte[]]$B, [int]$Start, [int]$End)
    $i = $Start; $name = ''; $dims = @(); $dtype = 0
    while ($i -lt $End) {
        $key = [int](Read-Varint $B ([ref]$i))
        $f = $key -shr 3; $w = $key -band 7
        if ($w -eq 2) {
            $len = [int](Read-Varint $B ([ref]$i))
            if ($f -eq 8) {
                $name = [System.Text.Encoding]::UTF8.GetString($B, $i, $len)
            }
            elseif ($f -eq 2) {
                # Dimension 消息：内部再取 field 1 (dim_value) 或 field 2 (dim_param)
                $j = $i; $end2 = $i + $len; $dval = $null; $dparam = $null
                while ($j -lt $end2) {
                    $k2 = [int](Read-Varint $B ([ref]$j))
                    $f2 = $k2 -shr 3; $w2 = $k2 -band 7
                    if ($f2 -eq 1 -and $w2 -eq 0) { $dval = Read-Varint $B ([ref]$j) }
                    elseif ($f2 -eq 2 -and $w2 -eq 2) {
                        $l2 = [int](Read-Varint $B ([ref]$j))
                        $dparam = [System.Text.Encoding]::UTF8.GetString($B, $j, $l2); $j += $l2
                    }
                    else { Skip-Field $B ([ref]$j) $w2 }
                }
                if ($null -ne $dval) { $dims += "$dval" }
                elseif ($null -ne $dparam) { $dims += $dparam }
                else { $dims += '?' }
            }
            $i += $len
        }
        elseif ($f -eq 1 -and $w -eq 0) { $dims += [int](Read-Varint $B ([ref]$i)) }
        elseif ($f -eq 2 -and $w -eq 0) { $dtype = [int](Read-Varint $B ([ref]$i)) }
        else { Skip-Field $B ([ref]$i) $w }
    }
    $dtName = switch ($dtype) { 1 {'FLOAT'} 7 {'INT64'} 6 {'INT32'} 9 {'BOOL'} 10 {'FLOAT16'} default {"type$dtype"} }
    return [pscustomobject]@{ Name = $name; Dims = ($dims -join ' x '); DType = $dtName }
}

function Read-GraphProto {
    param([byte[]]$B, [int]$Start, [int]$End)
    $i = $Start
    while ($i -lt $End) {
        $key = [int](Read-Varint $B ([ref]$i))
        $f = $key -shr 3; $w = $key -band 7
        if (($f -eq 11 -or $f -eq 12) -and $w -eq 2) {
            $len = [int](Read-Varint $B ([ref]$i))
            $label = if ($f -eq 11) { 'INPUT ' } else { 'OUTPUT' }
            $t = Read-TensorProto $B $i ($i + $len)
            Write-Host ("  {0}  name='{1}'  shape=[{2}]  dtype={3}" -f $label, $t.Name, $t.Dims, $t.DType)
            $i += $len
        }
        else { Skip-Field $B ([ref]$i) $w }
    }
}

$i = 0
Write-Host "=== ONNX 图结构: $(Split-Path $Path -Leaf) ===" -ForegroundColor Cyan
while ($i -lt $bytes.Length) {
    $key = [int](Read-Varint $bytes ([ref]$i))
    $f = $key -shr 3; $w = $key -band 7
    if ($f -eq 1 -and $w -eq 0) {
        $ir = Read-Varint $bytes ([ref]$i)
        Write-Host "ir_version = $ir"
    }
    elseif ($f -eq 7 -and $w -eq 2) {
        $len = [int](Read-Varint $bytes ([ref]$i))
        Read-GraphProto $bytes $i ($i + $len)
        $i += $len
    }
    else { Skip-Field $bytes ([ref]$i) $w }
}
Write-Host "=== 解析完成 ===" -ForegroundColor Cyan
