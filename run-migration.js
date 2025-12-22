import { Pool } from 'pg';
import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import dotenv from 'dotenv';

dotenv.config();

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

async function runMigration() {
  try {
    console.log('🔄 Reading migration file...');
    const sql = readFileSync(join(__dirname, 'migrations', '002_user_profiles.sql'), 'utf8');

    console.log('🚀 Executing user_profiles migration...');
    await pool.query(sql);

    console.log('✅ Migration completed successfully!');
    console.log('📊 user_profiles table created with fields:');
    console.log('   - id (PRIMARY KEY)');
    console.log('   - userId (REFERENCES user.id)');
    console.log('   - skill_level');
    console.log('   - software_background');
    console.log('   - hardware_background');
    console.log('   - learning_goal');
    console.log('   - createdAt, updatedAt');

    await pool.end();
    process.exit(0);
  } catch (error) {
    console.error('❌ Migration failed:', error.message);
    await pool.end();
    process.exit(1);
  }
}

runMigration();
