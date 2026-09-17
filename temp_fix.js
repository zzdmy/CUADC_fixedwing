const fs = require('fs');
const path = 'D:/vs26/CUADCcopter/main.cpp';
let data = fs.readFileSync(path, 'utf8');

const oldStr = 'last_fps_display = now;';
const newStr = `// RTK stats
                        if (rtkClient) {
                            auto& s = rtkClient->getStats();
                            std::cout << "RTK: valid=" << s.valid_frames.load()
                                << " invalid=" << s.invalid_frames.load()
                                << " pkt=" << s.mavlink_packets_sent.load()
                                << " " << s.total_bytes.load() << "B" << std::endl;
                        }
                        last_fps_display = now;`;

if (data.includes(oldStr)) {
    data = data.replace(oldStr, newStr);
    fs.writeFileSync(path, data);
    console.log('Replaced successfully');
} else {
    console.log('String not found');
}
