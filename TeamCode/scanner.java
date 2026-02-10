const fs = require('fs');
const path = require('path');

// Configuration: Change '.' to your specific folder path if needed
const TARGET_DIR = './';
        const EXCLUDED_FOLDERS = ['node_modules', '.git', 'dist', 'build'];
        const EXTENSIONS = ['.js', '.ts', '.html', '.css', '.json']; // Add what you need

let totalLines = 0;
let fileCount = 0;

function countLinesInDir(dir) {
    const files = fs.readdirSync(dir, { withFileTypes: true });

    for (const file of files) {
        const fullPath = path.join(dir, file.name);

        if (file.isDirectory()) {
            if (!EXCLUDED_FOLDERS.includes(file.name)) {
                countLinesInDir(fullPath);
            }
        } else if (EXTENSIONS.includes(path.extname(file.name))) {
            const content = fs.readFileSync(fullPath, 'utf-8');
            const lines = content.split('\n').length;
            totalLines += lines;
            fileCount++;
        }
    }
}

console.log('--- Scanning files... ---');
countLinesInDir(TARGET_DIR);

console.log(`\nResults:`);
console.log(`Total Files Scanned: ${fileCount}`);
        console.log(`Total Lines of Code: ${totalLines}`);