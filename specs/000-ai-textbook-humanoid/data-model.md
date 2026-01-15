# Data Model: AI-native Textbook Website for Physical AI & Humanoid Robotics

## Core Entities

### User
- `id`: UUID (Primary Key)
- `email`: String (Unique, Required)
- `name`: String (Required)
- `student_id`: String (Optional, for institutional tracking)
- `enrollment_date`: DateTime (Required)
- `current_week`: Integer (Default: 1)
- `progress_percentage`: Float (Default: 0.0)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

### CourseModule
- `id`: UUID (Primary Key)
- `title`: String (Required)
- `module_number`: Integer (Required, 1-4)
- `description`: Text (Required)
- `word_count`: Integer (Required, 3000-5000)
- `estimated_duration_hours`: Float (Required)
- `learning_outcomes`: JSON Array (Required)
- `prerequisites`: Array of UUIDs (Optional)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

### WeeklyContent
- `id`: UUID (Primary Key)
- `week_number`: Integer (Required, 1-13)
- `title`: String (Required)
- `module_id`: UUID (Foreign Key to CourseModule)
- `subtopics`: JSON Array (Required)
- `content_path`: String (Required, path to MD file)
- `exercises_count`: Integer (Required)
- `quizzes_count`: Integer (Required)
- `case_studies_count`: Integer (Required)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

### Exercise
- `id`: UUID (Primary Key)
- `title`: String (Required)
- `type`: String (Required: "coding", "simulation", "conceptual", "quiz")
- `difficulty`: String (Required: "beginner", "intermediate", "advanced")
- `content`: Text (Required)
- `solution`: Text (Required for auto-graded exercises)
- `module_id`: UUID (Foreign Key to CourseModule)
- `week_id`: UUID (Foreign Key to WeeklyContent, Optional)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

### StudentProgress
- `id`: UUID (Primary Key)
- `user_id`: UUID (Foreign Key to User)
- `module_id`: UUID (Foreign Key to CourseModule)
- `week_id`: UUID (Foreign Key to WeeklyContent, Optional)
- `exercise_id`: UUID (Foreign Key to Exercise, Optional)
- `status`: String (Required: "not_started", "in_progress", "completed")
- `score`: Float (Optional, 0.0-100.0)
- `attempts_count`: Integer (Default: 0)
- `last_accessed`: DateTime (Auto-generated)
- `completed_at`: DateTime (Optional)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

### ContentChunk
- `id`: UUID (Primary Key)
- `module_id`: UUID (Foreign Key to CourseModule)
- `chunk_index`: Integer (Required)
- `content`: Text (Required, chunk of MD content)
- `embedding_vector`: JSON Array (Required, for vector search)
- `semantic_tags`: JSON Array (Optional, for search optimization)
- `created_at`: DateTime (Auto-generated)

### ChatSession
- `id`: UUID (Primary Key)
- `user_id`: UUID (Foreign Key to User)
- `session_title`: String (Required)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)
- `is_active`: Boolean (Default: true)

### ChatMessage
- `id`: UUID (Primary Key)
- `session_id`: UUID (Foreign Key to ChatSession)
- `sender_type`: String (Required: "user", "assistant")
- `content`: Text (Required)
- `timestamp`: DateTime (Auto-generated)
- `context_chunks`: JSON Array (Optional, for RAG context)
- `message_type`: String (Optional: "query", "response", "exercise_help", "code_explanation")

### AssessmentProject
- `id`: UUID (Primary Key)
- `title`: String (Required)
- `project_number`: Integer (Required, 1-4)
- `description`: Text (Required)
- `rubric`: JSON Object (Required, evaluation criteria)
- `module_id`: UUID (Foreign Key to CourseModule, Optional)
- `deadline`: DateTime (Optional)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

### HardwareRequirement
- `id`: UUID (Primary Key)
- `name`: String (Required)
- `category`: String (Required: "workstation", "edge_computing", "robot_hardware")
- `specifications`: JSON Object (Required)
- `recommended_use`: Text (Optional)
- `cost_estimate`: String (Optional)
- `created_at`: DateTime (Auto-generated)
- `updated_at`: DateTime (Auto-generated)

## Relationships

1. **User** 1 → * **StudentProgress**: Users can have multiple progress entries across modules/weeks
2. **CourseModule** 1 → * **WeeklyContent**: Each module can have multiple weekly content pieces
3. **CourseModule** 1 → * **Exercise**: Each module can have multiple exercises
4. **WeeklyContent** 1 → * **Exercise**: Weekly content can have multiple exercises
5. **StudentProgress** 1 → * **Exercise**: Progress can track multiple exercises
6. **CourseModule** 1 → * **ContentChunk**: Each module has multiple content chunks for RAG
7. **User** 1 → * **ChatSession**: Each user can have multiple chat sessions
8. **ChatSession** 1 → * **ChatMessage**: Each session has multiple messages
9. **CourseModule** 1 → * **AssessmentProject**: Each module can have related projects

## Validation Rules

1. **User**: Email must be unique and valid format
2. **CourseModule**: Module_number must be between 1-4, word_count must be 3000-5000
3. **WeeklyContent**: Week_number must be between 1-13
4. **Exercise**: Difficulty must be one of allowed values
5. **StudentProgress**: Score must be between 0.0-100.0 if provided
6. **ContentChunk**: Must have embedding_vector for RAG functionality