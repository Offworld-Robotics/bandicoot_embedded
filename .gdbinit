target extended-remote :3333

set history save on

monitor reset halt

break main

define hook-quit
    set confirm off
end

load

run
